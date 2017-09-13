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
# OFED installation, includes opa_stack and OFED supplied ULPs

my $OFED_CONFIG = "$OFED_CONFIG_DIR/openib.conf";
my $default_prefix="/usr";

my $RPMS_SUBDIR = "RPMS";	# within ComponentInfo{'opa_stack'}{'SrcDir'}
my $SRPMS_SUBDIR = "SRPMS";	# within ComponentInfo{'opa_stack'}{'SrcDir'}

# list of components which are part of OFED
# can be a superset or subset of @Components
# ofed_vnic
my @ofed_components = ( 
				"opa_stack", 		# Kernel drivers.
				"intel_hfi", 		# HFI drivers
				"ib_wfr_lite",		# ib_wfr_lite kernel module 
				"ofed_mlx4", 		# MLX drivers
				"opa_stack_dev", 	# dev libraries.
				"ofed_ipoib", 		# ipoib module.
				"ofed_ib_bonding", 	# ib_bonding rpm is separate from opa_stack.
				"ofed_rds",  		# Now just tools. 
				"ofed_udapl", "ofed_udaplRest", 
				"ofed_srp", 		# SRP module.
				"ofed_srpt", 		# SRP target module.
				"ofed_iser",		# iSCSI over RDMA
				"mpi_selector", 	
				"mvapich2", 
				"openmpi",
				"gasnet",
				"openshmem",
				"ofed_mpisrc", 		# Source bundle for MPIs.
				"mpiRest",			# PGI, Intel mpi variants.
				"ofed_iwarp", 		# iwarp support.
				"opensm", 			# OFED SM.
				"ofed_nfsrdma",		# Enabling NFS over RDMA.
				"ofed_debug", # must be last real component
				"ibacm", 			# OFA IB communication manager assistant.
);

# information about each component in ofed_components
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
#	Drivers => used so we can load ifs-kernel-updates rpm, then remove drivers for
#				ULPs which are not desired.  Ugly but a workaround for fact
#				that ifs-kernel-updates has all the ULP drivers in it
#				specifies drivers and subdirs which are within ifs-kernel-updates and
#				specific to each component
#	StartupScript => name of startup script which controls startup of this
#					component
#	StartupParams => list of parameter names in $OFED_CONFIG which control
#					startup of this component (set to yes/no values)
#
# Note KernelRpms are always installed before UserRpms
my %ofed_comp_info = (
	'opa_stack' => {
					KernelRpms => [ "ifs-kernel-updates" ], # special case
					UserRpms =>	  [ "ifs-scripts", "ofed-scripts",
									"libibverbs", "libibverbs-utils",
									"libibumad",
									"libibmad",
									"libibcm",
									"librdmacm", "librdmacm-utils",
									#"libmthca",			# Obsolete
									"libipathverbs",
									#"libehca",				# Broken in 3.9.2.
									#"libibcommon-compat", 	# Obsolete
									#"libibumad-compat", 	# Obsolete
									#"libibumad-compat-2", 	# Obsolete
									#"libibmad-compat", 	# Obsolete
									"ibacm",
									#"infinipath-psm",
# TBD - in future OFED dependency on opensm-libs will go away and can
# move opensm-libs into opensm component
									"opensm-libs",
									"ofed-docs",
# TBD move these to a performance tests package
									"perftest", "qperf",
								  ],
					DebugRpms =>  [ "libibverbs-debuginfo",
									#"libibcommon-debuginfo", # Obsolete
									"libibumad-debuginfo",
									"libibmad-debuginfo",
									"libibcm-debuginfo",
									"librdmacm-debuginfo",
									"libmthca-debuginfo",
									"libipathverbs-debuginfo",
									#"libehca-debuginfo",	# broken.
									"qperf-debuginfo",
									#"infinipath-psm-debuginfo",
								  ],
					Drivers => "", 	# we don't need to list these here
							# ib stack startup is controlled by
							# ONBOOT and openibd script
							# other parameters are left enabled so we can
							# manually start the stack
							# those include: RDMA_CM_LOAD, RDMA_UCM_LOAD,
							# MTHCA_LOAD, IPATH_LOAD, QIB_LOAD, KCOPY_LOAD
							# MLX4_EN_LOAD, UCM_LOAD
							# and perhaps future ehca flags
					StartupScript => "openibd",
					StartupParams => [ "ONBOOT", "RDMA_UCM_LOAD" ],
					},

	'intel_hfi' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "hfi1", "hfi1-devel",
							    "libhfi1verbs", "libhfi1verbs-devel",
							    "hfi-utils", 
							    "hfi1-psm", 
							    "hfi1-psm-devel", "hfi1-psm-compat",
							    "hfi1-diagtools-sw",
							    "hfi-firmware" 
					    ],
					DebugRpms =>  [ "hfi1_debuginfo", "hfi1-psm-devel-noship", 
							"hfi1-diagtools-sw-debuginfo"
					    ],
					Drivers => "",
					StartupScript => "openibd",
					StartupParams => [  ],
					},
	'ib_wfr_lite' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "ib_wfr_lite", ],
					DebugRpms =>  [ "ib_wfr_lite-debuginfo", ],
					Drivers => "",
					StartupScript => "openibd",
					StartupParams => [  ],
			      },
	'ofed_mlx4' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "libmlx4",
									"libmlx4-devel",
								  ],
					DebugRpms =>  [ "libmlx4-debuginfo",
								  ],
					Drivers => "mlx4_ib drivers/infiniband/hw/mlx4
						 		mlx4_core drivers/net/mlx4",
					StartupScript => "openibd",
					# MWHEINZ FIXME StartupParams => [ "MLX4_LOAD" ],
					StartupParams => [  ],
					},
	'opa_stack_dev' => {
					KernelRpms => [  ],
					UserRpms =>	  [ "ibacm-devel", 
							    "ifs-kernel-updates-devel",
							    "ifs-kernel-updates-scripts",
									"libibverbs-devel",
									"libibverbs-devel-static",
									#"libibcommon-devel", "libibcommon-static",	# Obsolete
									"libibumad-devel", "libibumad-static",
									"libibmad-devel", "libibmad-static",
									"libibcm-devel",
									"librdmacm-devel",
									#"libmthca-devel-static",	# Obsolete
									"libipathverbs-devel",
									#"libehca-devel-static",	# broken
									#"infinipath-psm-devel",
# TBD - in future OFED dependency on opensm-libs will go away and can
# move opensm-devel and opensm-static into opensm component and perhaps ibsim
									"opensm-devel", "opensm-static",
									"ibsim",
								  ],
					DebugRpms =>  [ "ibsim-debuginfo", ],
					Drivers => "", 	# none
					StartupScript => "",
					StartupParams => [ ],
					},
	'ofed_ipoib' => {
					KernelRpms => [ ],
					UserRpms =>	  [ ],
					DebugRpms =>  [ ],
					Drivers => "ib_ipoib drivers/infiniband/ulp/ipoib",
					StartupScript => "openibd",
					StartupParams => [ "IPOIB_LOAD" ],
					},
	'ofed_ib_bonding' => {
					# code assumes ib-bonding is only Rpm
					KernelRpms => [ "ib-bonding", ],
					UserRpms =>	  [ ],
					DebugRpms =>  [ "ib-bonding-debuginfo", ],
					Drivers => "", # none from ifs-kernel-updates
					StartupScript => "",
					StartupParams => [ ],
					},
	'ofed_rds' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "rds-tools", "rds-devel", ],
					DebugRpms =>  [ ],
					Drivers => "rds net/rds",
					StartupScript => "openibd",
					StartupParams => [ "RDS_LOAD" ],
					},
	'ofed_udapl' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "dapl", "dapl-devel",
									"dapl-devel-static",
									"dapl-utils",
								  ],
					DebugRpms =>  [ "dapl-debuginfo", ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
	# ofed install does not install these
	# due to conflicts can't install both this and dapl-2 versions
	'ofed_udaplRest' => {
					KernelRpms => [ ],
					UserRpms =>	  [ ],
					DebugRpms =>  [ ],	# leave in UserRpms not used
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
	'ofed_srp' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "srptools", ],
					Drivers => "ib_srp drivers/infiniband/ulp/srp",
					DebugRpms =>  [ ],
					StartupScript => "openibd",
					#MWHEINZ DISABLED FOR WFR-LITE WORK StartupParams => [ "SRP_LOAD" ],
					StartupParams => [ ],
					},
	'ofed_srpt' => {
					KernelRpms => [ ],
					UserRpms =>	  [ ],
					DebugRpms =>  [ ],
					Drivers => "ib_srpt drivers/infiniband/ulp/srpt",
					StartupScript => "openibd",
					StartupParams => [ "SRPT_LOAD" ],
					},
	'ofed_iser' => {
					# starting with 1.5.2rc3 iser no longer needed OFED iscsi
					KernelRpms => [ ],
					UserRpms =>	  [ ],
					DebugRpms =>  [ ],
					Drivers => "ib_iser drivers/infiniband/ulp/iser",
					StartupScript => "openibd",
					#MWHEINZ FIXME StartupParams => [ "ISER_LOAD" ],
					StartupParams => [ ],
					},
	'ofed_nfsrdma' => {
					KernelRpms => [ ],
					UserRpms =>	  [ ],
					DebugRpms =>  [ ],
					Drivers => "auth_rpcgss net/sunrpc/auth_gss
								rpcsec_gss_krb5 net/sunrpc/auth_gss
								rpcsec_gss_spkm3 net/sunrpc/auth_gss
								exportfs fs/exportfs
								lockd fs/lockd
								nfs_acl fs/nfs_common
								nfsd fs/nfsd
								nfs fs/nfs
								sunrpc net/sunrpc
								svcrdma net/sunrpc/xprtrdma
								xprtrdma net/sunrpc/xprtrdma",
					StartupScript => "",	# not affected by OFED install
					StartupParams => [ ],
					},
	'mpi_selector' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "mpi-selector" ],
					DebugRpms =>  [ ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
	'mvapich2' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "mvapich2_gcc", "mpitests_mvapich2_gcc" ],
					DebugRpms =>  [ ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
	'openmpi' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "openmpi_gcc_hfi", "mpitests_openmpi_gcc_hfi" , "openmpi_gcc", "mpitests_openmpi_gcc" ],
					DebugRpms =>  [ ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
	'gasnet' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "gasnet_gcc_hfi", "gasnet_gcc_hfi-devel" , "gasnet_gcc_hfi-tests" ],
					DebugRpms =>  [ ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
	'openshmem' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "openshmem_gcc_hfi", "openshmem-test-suite_gcc_hfi"  ],
					DebugRpms =>  [ ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
	'ofed_mpisrc' => {	# nothing to build, just copies srpms
					KernelRpms => [ ],
					UserRpms =>	  [ ],
					DebugRpms =>  [ ],
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
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
	'ofed_iwarp' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "libcxgb3",
									"libcxgb3-devel",
									"libcxgb4",
									"libcxgb4-devel",
									"libnes",
									"libnes-devel-static",
								  ],
					DebugRpms =>  [ "libcxgb3-debuginfo",
									"libcxgb4-debuginfo",
									"libnes-debuginfo",
								  ],
					Drivers => "iw_cxgb3 drivers/infiniband/hw/cxgb3
						 		cxgb3 drivers/net/cxgb3
								iw_cxgb4 drivers/infiniband/hw/cxgb4
						 		cxgb4 drivers/net/cxgb4
						 		iw_nes drivers/infiniband/hw/nes",
					StartupScript => "openibd",
					#MWHEINZ FIXME 
					#StartupParams => [ "CXGB3_LOAD", "CXGB4_LOAD", "NES_LOAD" ],
					StartupParams => [ ],
					},
	'opensm' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "opensm", ],
					DebugRpms =>  [ "opensm-debuginfo", ],
					Drivers => "", # none
					StartupScript => "opensmd",
					StartupParams => [ ],
					},
	'ofed_debug' => {
					KernelRpms => [ ],
					UserRpms =>	  [ ],
					DebugRpms =>  [ ],	# listed by comp above
					Drivers => "", # none
					StartupScript => "",
					StartupParams => [ ],
					},
	'ibacm' => {
					KernelRpms => [ ],
					UserRpms =>	  [ "ibacm", ],
					DebugRpms =>  [ ],
					Drivers => "", # none
					StartupScript => "ibacm",
					StartupParams => [ ],
					},
);

# options for building ifs-kernel-updates srpm and what platforms they
# are each available for
my %ofed_kernel_ib_options = (
	# build option		# arch & kernels supported on
						# (see util_build.pl for more info on format)
	# NOTE: the ifs-kernel-updates build takes no arguments right now.
	# This list is now empty, but will probably get used as time goes on.
);

# all kernel srpms
# these are in the order we must build/process them to meet basic dependencies
my @ofed_kernel_srpms = ( 'ifs-kernel-updates', 'ib-bonding' );

# all user space srpms
# these are in the order we must build/process them to meet basic dependencies
my @ofed_user_srpms = (
		"ifs-scripts", "ofed-scripts", "libibverbs", 
		#"libmthca", 
		"libmlx4", "libcxgb3", "libcxgb4", "libnes", "libipathverbs", #"libehca",
	   	#"libibcommon",
	   	"libibumad", "libibmad", "libibcm", "librdmacm", 
		"opensm", "ibsim",
		#"libibcommon-compat", libibumad-compat", "libibumad-compat-2", libibmad-compat",
#		"infinipath-psm",
		"ibacm", "perftest", "mstflint",
		"srptools", "rds-tools", 
		"infiniband-diags", "qperf",
		"ofed-docs", "mpi-selector", "dapl",
		"hfi1", "libhfi1verbs", "hfi-utils", "hfi1-psm", "hfi1-diagtools-sw", "hfi-firmware",
		"ib_wfr_lite",
 		"mvapich2", "openmpi", "gasnet", "openshmem", "openshmem-test-suite"
);

# rpms not presently automatically built
my @ofed_other_srpms = ( );

my @ofed_srpms = ( @ofed_kernel_srpms, @ofed_user_srpms, @ofed_other_srpms );

# This provides information for all kernel and user space srpms
# Fields:
#	Available => indicate which platforms each srpm can be built for
#	PostReq => after building each srpm, some of its generated rpms will
#				need to be installed so that later srpms will build properly
#				this lists all the rpms which may be needed by subsequent srpms
#	Builds => list of kernel and user rpms built from each srpm
#				caller must know if user/kernel rpm is expected
#	PartOf => ofed_components this srpm is part of
#			  this is filled in at runtime based on ofed_comp_info
#			  cross referenced to Builds
#	BuildPrereq => OS prereqs needed to build this srpm
#				List in the form of "rpm version mode" for each.
#				version is optional, default is "any"
#				mode is optional, default is "any"
#				Typically mode should be "user" for libraries
#				See rpm_is_installed for more information about mode
my %ofed_srpm_info = (
	"ifs-kernel-updates" =>	{ Available => "",
					  Builds => "ifs-kernel-updates ifs-kernel-updates-devel ifs-kernel-updates-scripts",
					  PostReq => "ifs-kernel-updates ifs-kernel-updates-devel ifs-kernel-updates-scripts",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"ib-bonding" =>	{ Available => "",
					  Builds => "ib-bonding ib-bonding-debuginfo",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"hfi1" =>	{ Available => "",
					  Builds => "hfi1 hfi1-devel",
					  PostReq => "hfi1-devel",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"hfi-utils" =>	{ Available => "",
					  Builds => "hfi-utils",
					  PostReq => "hfi-utils",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"hfi1-psm" =>	{ Available => "",
					  Builds => "hfi1-psm hfi1-psm-devel hfi1-psm-devel-noship hfi1-psm-compat",
					  PostReq => "hfi1-psm hfi1-psm-devel hfi1-psm-devel-noship hfi1-psm-compat",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"libhfi1verbs" =>	{ Available => "",
					  Builds => "libhfi1verbs libhfi1verbs-devel",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"hfi1-diagtools-sw" =>	{ Available => "",
					  Builds => "hfi1-diagtools-sw hfi1-diagtools-sw-debuginfo",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'readline-devel', 'ncurses-devel', ],
					},
	"hfi-firmware" =>	{ Available => "",
					  Builds => "hfi-firmware",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"ib_wfr_lite" =>	{ Available => "",
					  Builds => "ib_wfr_lite",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"ifs-scripts" => { Available => "",
					  Builds => "ifs-scripts",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"ofed-scripts" => { Available => "",
					  Builds => "ofed-scripts",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"libibverbs" =>	{ Available => "",
					  Builds => "libibverbs libibverbs-devel libibverbs-devel-static libibverbs-utils libibverbs-debuginfo",
					  PostReq => "libibverbs libibverbs-devel libibverbs-devel-static",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'gcc 4.1.2', 'glibc-devel any user', 'libstdc++ any user' ],
					},
	"libmlx4" =>	{ Available => "",
					  Builds => "libmlx4 libmlx4-devel libmlx4-debuginfo",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"libcxgb3" => 	{ Available => "",
					  Builds => "libcxgb3 libcxgb3-devel libcxgb3-debuginfo",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"libcxgb4" => 	{ Available => "",
					  Builds => "libcxgb4 libcxgb4-devel libcxgb4-debuginfo",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"libnes" => 	{ Available => "",
					  Builds => "libnes libnes-devel-static libnes-debuginfo",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"libipathverbs" => { Available => "",
					  Builds => "libipathverbs libipathverbs-devel libipathverbs-debuginfo",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"libibumad" =>	{ Available => "",
					  Builds => "libibumad libibumad-devel libibumad-static libibumad-debuginfo",
					  PostReq => "libibumad libibumad-devel",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'libtool any user' ],
					},
	"libibmad" =>	{ Available => "",
					  Builds => "libibmad libibmad-devel libibmad-static libibmad-debuginfo",
					  PostReq => "libibmad libibmad-devel",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'libtool any user' ],
					},
	"libibumad-compat-2" =>   { Available => "",
					  Builds => "libibumad-compat-2",
					  PostReq => "libibumad-compat-2",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"libibcm" =>	{ Available => "",
					  Builds => "libibcm libibcm-devel libibcm-debuginfo",
					  PostReq => "libibcm libibcm-devel",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"ibacm" =>		{ Available => "",
					  Builds => "ibacm ibacm-devel",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"librdmacm" =>	{ Available => "",
					  Builds => "librdmacm librdmacm-devel librdmacm-debuginfo librdmacm-utils",
					  PostReq => "librdmacm librdmacm-devel",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"opensm" =>		{ Available => "",
					  Builds => "opensm-libs opensm opensm-devel opensm-static opensm-debuginfo",
					  PostReq => "opensm-libs opensm-devel",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'bison', 'flex' ],
					},
	"ibsim" =>		{ Available => "",
					  Builds => "ibsim ibsim-debuginfo",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"dapl" =>	{ Available => "",
					  Builds => "dapl dapl-devel dapl-devel-static dapl-utils dapl-debuginfo",
					  PostReq => "dapl dapl-devel dapl-devel-static",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"infinipath-psm" => { Available => "",
					  Builds => "infinipath-psm infinipath-psm-devel infinipath-psm-debuginfo",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"perftest" =>	{ Available => "",
					  Builds => "perftest",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"srptools" =>	{ Available => "",
					  Builds => "srptools",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"rds-tools" =>	{ Available => "",
					  Builds => "rds-tools rds-devel",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"qperf" =>		{ Available => "",
					  Builds => "qperf qperf-debuginfo",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"ofed-docs" =>	{ Available => "",
					  Builds => "ofed-docs",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
	"mpi-selector" => { Available => "",
					  Builds => "mpi-selector",
					  PostReq => "mpi-selector",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'tcsh' ],
					},
	"mvapich2" =>	{ Available => "",
						# mpitests are built by do_mvapich2_build
		 			  Builds => "mvapich2_gcc mpitests_mvapich2_gcc",
		 			  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'libstdc++ any user',
					  				   'libstdc++-devel any user',
									   'sysfsutils any user',
					  				   'g77', 'libgfortran any user'
								   	],
					},
	"openmpi" =>	{ Available => "",
						# mpitests are built by do_openmpi_build
		 			  Builds => "openmpi_gcc_hfi mpitests_openmpi_gcc_hfi openmpi_gcc mpitests_openmpi_gcc",
		 			  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ 'libstdc++ any user',
					  				   'libstdc++-devel any user',
					  				   'g77', 'libgfortran any user',
									   'binutils'
								   	],
					},
	"gasnet" =>	{ Available => "",
		 			  Builds => "gasnet_gcc_hfi gasnet_gcc_hfi-devel gasnet_gcc_hfi-tests",
		 			  PostReq => "gasnet_gcc_hfi gasnet_gcc_hfi-devel",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ ],
					},
	"openshmem" =>	{ Available => "",
		 			  Builds => "openshmem_gcc_hfi",
		 			  PostReq => "openshmem_gcc_hfi",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [ ],
					},
	"openshmem-test-suite" =>	{ Available => "",
					  Builds => "openshmem-test-suite_gcc_hfi",
					  PostReq => "",
					  PartOf => "", # filled in at runtime
					  BuildPrereq => [],
					},
);

# This provides information for all kernel and user space rpms
# This is built based on information in ofed_srpm_info and ofed_comp_info
# Fields:
#	Available => boolean indicating if rpm can be available for this platform
#	Parent => srpm which builds this rpm
#	PartOf => space separate list of components this rpm is part of
#	Mode => mode of rpm (kernel or user)
my %ofed_rpm_info = ();
my @ofed_rpms = ();

# RPMs from previous versions of OFED
# as new RPMs are added above for present release, also add them here
my @ofed1_2_5_and_1_3_release_rpms = (
			"kernel-ib",
			"ofed-scripts",
 			"libibverbs", "libibverbs-utils", "libibverbs-debuginfo",
			"libibcommon", "libibcommon-debuginfo",
			"libibumad", "libibumad-debuginfo",
			"libibmad", "libibmad-debuginfo",
			"libibcm", "libibcm-debuginfo",
			"librdmacm", "librdmacm-utils", "librdmacm-debuginfo",
			"libmlx4", "libmlx4-debuginfo",
			"libmthca", "libmthca-debuginfo",
			"libipathverbs", "libipathverbs-debuginfo",
			"libehca", "libehca-debuginfo",
			"libosmcomp", "libosmvendor", "libopensm", 
			"opensm-libs",
			"mstflint", "tvflash",
			"ofed-docs",
			"perftest", "qperf", "qperf-debuginfo",
			"ibutils", "infiniband-diags", "openib-diags",

			"kernel-ib-devel",
			"libibverbs-devel", "libibverbs-devel-static",
			"libibcommon-devel", "libibcommon-static",
			"libibumad-devel", "libibumad-static",
			"libibmad-devel", "libibmad-static",
			"libibcm-devel",
			"librdmacm-devel",
			"libmlx4-devel-static",
			"libmthca-devel-static",
			"libipathverbs-devel",
			"libehca-devel-static",
			"opensm-devel", "opensm-static",
			"libosmcomp-devel", "libosmvendor-devel", "libopensm-devel",
			"ibsim", "ibsim-debuginfo",
			"ib-bonding", "ib-bonding-debuginfo", "ipoibtools",
			"rds-tools",
			"dapl", "dapl-devel", "dapl-devel-static",
			"dapl-utils", "dapl-debuginfo",

			# redhat renamed 1.2.5 OFED as follows:
			"compat-dapl-devel-1.2.5", "compat-dapl-static-1.2.5",
			"compat-dapl-1.2.5",
			"dapl-static", "dapl-devel", "dapl",
 			"librdmacm-static", "libibcm-static", "libibverbs-static",
			"libmlx4-static", "libmthca-static", "libipathverbs-static",
			"libcxgb3-static", "libcxgb4-static", "libnes-static",

			"srptools", "ibvexdmtools", "qlvnictools", "mpi-selector",
			"mvapich2", "mvapich2_gcc", "mvapich2_pgi", "mvapich2_intel",
			"mvapich2_pathscale",
			"openmpi", "openmpi_gcc", "openmpi_pgi", "openmpi_intel",
			"openmpi_pathscale",
			"mpitests", 
			"mpitests_mvapich2",
			"mpitests_mvapich2_gcc", "mpitests_mvapich2_pgi",
			"mpitests_mvapich2_intel", "mpitests_mvapich2_pathscale",
			"mpitests_openmpi",
			"mpitests_openmpi_gcc", "mpitests_openmpi_pgi",
			"mpitests_openmpi_intel", "mpitests_openmpi_pathscale",
			"libcxgb3", "libcxgb3-debuginfo", "libcxgb3-devel",
			"libcxgb4", "libcxgb4-debuginfo", "libcxgb4-devel",
			"libnes", "libnes-debuginfo", "libnes-devel-static", "libnes-devel",
			"opensm", "opensm-debuginfo",
);
# new ones since 1.3
my @ofed1_4_release_rpms = (
			"compat-dapl", "compat-dapl-devel", "compat-dapl-devel-static",
		   	"compat-dapl-utils", "compat-dapl-debuginfo",
			"libmlx4-devel",
		   	# "tgt-generic", "tgt",
			"scsi-target-utils", 
			"tvflash-debuginfo",
			# a few which RHEL 5.3 chose to rename
			"compat-dapl-static", "mpitests-mvapich2", "mpitests-openmpi",
);
my @ofed1_4_1_release_rpms = (
			"rnfs-utils", "rnfs-utils-debuginfo",
);
my @ofed1_5_2_release_rpms = (
			"ibacm", "infinipath-psm", "infinipath-psm-devel",
);
# A list of the OFED rpms that PathScale/QLogic may have distributed in previous
# releases.  Omits rpms already listed above
my @qlgc_old_rpms = ( "rhel4-ofed-fixup", "qlogic-mpi-register",
			"infinipath-kernel", "infinipath", "infinipath-libs",
			"infinipath-doc", "infinipath-devel",
			"mpi-frontend", "mpi-benchmark", "mpi-libs", "mpi-doc", "mpi-devel", 
			"tmi", "openmpi_pgi_qlc", "openmpi_pathscale_qlc", 
			"openmpi_intel_qlc", "openmpi_gcc_qlc",
			"mvapich2_pgi_qlc", "mvapich2_pathscale_qlc", "mvapich2_intel_qlc",
			"mvapich2_gcc_qlc",
			"mpitests_openmpi_pgi_qlc", "mpitests_openmpi_pathscale_qlc",
			"mpitests_openmpi_intel_qlc", "mpitests_openmpi_gcc_qlc",
			"mpitests_mvapich2_pgi_qlc", "mpitests_mvapich2_pathscale_qlc",
			"mpitests_mvapich2_intel_qlc", "mpitests_mvapich2_gcc_qlc",
			"mpi-noship-tests",
			);
my @sles11_ofed_rpms = (
			# these are in SLES11 and are renames of some OFED 1.4 rpms
			"libcxgb3-rdmav2", "libcxgb4-rdmav2",
		   	"libmthca-rdmav2", "libmlx4-rdmav2",
			"libcxgb3-rdmav2-devel", "libcxgb4-rdmav2-devel",
		   	"libmthca-rdmav2-devel", "libmlx4-rdmav2-devel",
			"libibcommon1", "libibumad1", "libibmad1",
		   	"ofed-1.4.0", "ofed-1.4.1", "ofed-1.4.2", "ofed-1.5.2",
			"ofa",
			"libnes-rdmav2", "libamso-rdmav2",
			"libnes-rdmav2-devel", "libamso-rdmav2-devel",
			"ofed-doc", "ofed-kmp", "ofed-kmp-smp", "ofed-kmp-default",
		   	"ofed-kmp-debug", "ofed-kmp-kdump",
		   	"ofed-cxgb3-NIC-kmp-smp", "ofed-cxgb3-NIC-kmp-default",
		   	"ofed-cxgb3-NIC-kmp-debug", "ofed-cxgb3-NIC-kmp-kdump",
		   	"ofed-cxgb4-NIC-kmp-smp", "ofed-cxgb4-NIC-kmp-default",
		   	"ofed-cxgb4-NIC-kmp-debug", "ofed-cxgb4-NIC-kmp-kdump",
			# given naming convention, these MIGHT appear in SLES11 at some time
			"libipathverbs-rdmav2", "libehca-rdmav2",
			"libipathverbs-rdmav2-devel", "libehca-rdmav2-devel",
			);
my @prev_release_rpms = (
			@ofed1_2_5_and_1_3_release_rpms, @ofed1_4_release_rpms,
			@ofed1_4_1_release_rpms, @ofed1_5_2_release_rpms,
			@qlgc_old_rpms, @sles11_ofed_rpms,
			"rdma-ofa-agent", "libibumad3", "libibmad5",
			"mpich_mlx", "ibtsal", "openib", "opensm", "opensm-devel", "opensm-libs",
			"mpi_osu", "mpi_ncsa", "thca", "ib-osm", "osm", "diags", "ibadm", "ib-diags",
			"ibgdiag", "ibdiag", "ib-management",
			"ib-verbs", "ib-ipoib", "ib-cm", "ib-sdp", "ib-dapl", "udapl",
			"udapl-devel", "libdat", "libibat", "ib-kdapl", "ib-srp",
			"ib-srp_target", "oiscsi-iser-support", "ofed-docs", "ofed-scripts"
);

# RPMs from OFED in Redhat
my @redhat_rpms = (
			"udapl", "udapl-devel", "dapl", "dapl-devel", "libibcm",
			"libibcm-devel", "libibcommon", "libibcommon-devel",
			"libibmad", "libibmad-devel", "libibumad", "libibumad-devel",
			"libibverbs", "libibverbs-devel", "libibverbs-utils",
			"libipathverbs", "libipathverbs-devel", "libmthca",
			"libmthca-devel", "libmlx4", "libmlx4-devel",
			"libehca", "libehca-devel",
			"libsdp", "librdmacm", "librdmacm-devel", "librdmacm-utils",
			"openib",
			"openib-diags", "openib-mstflint", "openib-perftest",
			"openib-srptools", "openib-tvflash",
			"openmpi", "openmpi-devel", "openmpi-libs", "openmpi11",
		   	"opensm", "opensm-devel", "opensm-libs", "compat-opensm-libs",
			"ibutils", "ibutils2", "ibutils-devel", "ibutils2-devel", "ibutils-libs",
			"rdma",
			"compat-openmpi", "compat-openmpi-psm",
		   	"mvapich-psm", "mvapich-psm-devel", "mpitests-mvapich-psm",
		   	"mvapich2-psm", "mvapich2-psm-devel", "mpitests-mvapich2-psm",
		   	"openmpi-psm", "openmpi-psm-devel", "mpitests-openmpi-psm",
);

# RPMs from OFED in SuSE
my @suse_rpms = (
			"libamso", "libamso-devel", "dapl2", "dapl2-devel", "mvapich2",
		   	"mvapich2-devel", "mvapich-devel"
);

my %ofed_autostart_save = ();
# ==========================================================================
# OFED opa_stack build in prep for installation

# based on %ofed_srpm_info{}{'Available'} determine if the given SRPM is
# buildable and hence available on this CPU for $osver combination
# "user" and kernel rev values for mode are treated same
sub available_srpm($$$)
{
	my $srpm = shift();
	# $mode can be user or any other value,
	# only used to select Available 
	my $mode = shift();	# "user" or kernel rev
	my $osver = shift();
	my $avail = "Available";

	DebugPrint("checking $srpm $mode $osver against '$ofed_srpm_info{$srpm}{$avail}'\n");
	return arch_kernel_is_allowed($osver, $ofed_srpm_info{$srpm}{$avail});
}

# initialize ofed_rpm_info based on specified osver for present system
sub init_ofed_rpm_info($)
{
	my $osver = shift();

	%ofed_rpm_info = ();	# start fresh

	# first get information based on all srpms
	foreach my $srpm ( @ofed_srpms ) {
		my @package_list = split /[[:space:]]+/, $ofed_srpm_info{$srpm}{'Builds'};
		foreach my $package ( @package_list ) {
			next if ( "$package" eq '' );	# handle leading spaces
			$ofed_rpm_info{$package}{'Available'} = available_srpm($srpm, "user", $osver);
			$ofed_rpm_info{$package}{'Parent'} = "$srpm";
		}
	}

	# now fill in PartOf and Mode based on all components
	# allow for the case where a package could be part of more than 1 component
	# however assume User vs Kernel is consistent
	foreach my $comp ( @ofed_components ) {
		foreach my $package ( @{ $ofed_comp_info{$comp}{'UserRpms'}} ) {
			$ofed_rpm_info{$package}{'PartOf'} .= " $comp";
			$ofed_rpm_info{$package}{'Mode'} = "user";
		}
		foreach my $package ( @{ $ofed_comp_info{$comp}{'KernelRpms'}} ) {
			$ofed_rpm_info{$package}{'PartOf'} .= " $comp";
			$ofed_rpm_info{$package}{'Mode'} = "kernel";
		}
		foreach my $package ( @{ $ofed_comp_info{$comp}{'DebugRpms'}} ) {
			$ofed_rpm_info{$package}{'PartOf'} .= " ofed_debug";
			# this is the only debuginfo with kernel rev in its name
			if ( "$package" eq "ib-bonding-debuginfo" ) {
				$ofed_rpm_info{$package}{'Mode'} = "kernel";
			} else {
				$ofed_rpm_info{$package}{'Mode'} = "user";
			}
		}
	}
	@ofed_rpms = ( keys %ofed_rpm_info );	# list of all rpms
	# fixup Available for those in "*Rest" but not in ofed_srpm_info{*}{Builds}
	foreach my $package ( @ofed_rpms ) {
		if ("$ofed_rpm_info{$package}{'Available'}" eq "" ) {
			$ofed_rpm_info{$package}{'Available'}=0;
		}
	}

	# -debuginfo is not presently supported on SuSE or Centos
	# also .rpmmacros overrides can disable debuginfo availability
	if ( "$CUR_DISTRO_VENDOR" eq "SuSE"
		|| ! rpm_will_build_debuginfo()) {
		foreach my $package ( @ofed_rpms ) {
			if ($package =~ /-debuginfo/) {
				$ofed_rpm_info{$package}{'Available'} = 0;
		}
	}
	}

	# every package must be part of some component (could be a dummy component)
	foreach my $package ( @ofed_rpms ) {
		if ( "$ofed_rpm_info{$package}{'PartOf'}" eq "" ) {
			LogPrint "$package: Not PartOf anything\n";
		}
	}

	# build $ofed_srpm_info{}{PartOf}
	foreach my $srpm ( @ofed_srpms ) {
		$ofed_srpm_info{$srpm}{'PartOf'} = '';
	}
	foreach my $package ( @ofed_rpms ) {
		my @complist = split /[[:space:]]+/, $ofed_rpm_info{$package}{'PartOf'};
		my $srpm = $ofed_rpm_info{$package}{'Parent'};
		foreach my $comp ( @complist ) {
			next if ( "$comp" eq '' );	# handle leading spaces
			if ( " $ofed_srpm_info{$srpm}{'PartOf'} " !~ / $comp / ) {
				$ofed_srpm_info{$srpm}{'PartOf'} .= " $comp";
			}
		}
	}
	# fixup special case for ifs-kernel-updates, its part of all ofed comp w/ Drivers
	my $srpm = $ofed_rpm_info{'ifs-kernel-updates'}{'Parent'};
	foreach my $comp ( @ofed_components ) {
		if ("$ofed_comp_info{$comp}{'Drivers'}" ne "" ) {
			if ( " $ofed_srpm_info{$srpm}{'PartOf'} " !~ / $comp / ) {
				$ofed_srpm_info{$srpm}{'PartOf'} .= " $comp";
			}
		}
	}

	if (DebugPrintEnabled() ) {
		# dump all SRPM info
		DebugPrint "\nSRPMs:\n";
		foreach my $srpm ( @ofed_srpms ) {
			DebugPrint("$srpm => Builds: '$ofed_srpm_info{$srpm}{'Builds'}'\n");
			DebugPrint("           PostReq: '$ofed_srpm_info{$srpm}{'PostReq'}'\n");
			DebugPrint("           Available: '$ofed_srpm_info{$srpm}{'Available'}'\n");
			DebugPrint("           Available: ".available_srpm($srpm, "user", $osver)." PartOf '$ofed_srpm_info{$srpm}{'PartOf'}'\n");
		}

		# dump all RPM info
		DebugPrint "\nRPMs:\n";
		foreach my $package ( @ofed_rpms ) {
			DebugPrint("           Mode: $ofed_rpm_info{$package}{'Mode'} PartOf: '$ofed_rpm_info{$package}{'PartOf'}'\n");
		}

		# dump all OFED component info
		DebugPrint "\nOFA Components:\n";
		foreach my $comp ( @ofed_components ) {
			DebugPrint("   $comp: KernelRpms: @{ $ofed_comp_info{$comp}{'KernelRpms'}}\n");
			DebugPrint("           UserRpms: @{ $ofed_comp_info{$comp}{'UserRpms'}}\n");
			DebugPrint("           DebugRpms: @{ $ofed_comp_info{$comp}{'DebugRpms'}}\n");
			DebugPrint("           Drivers: $ofed_comp_info{$comp}{'Drivers'}\n");
			DebugPrint("           StartupParams: @{ $ofed_comp_info{$comp}{'StartupParams'}}\n");
		}
		DebugPrint "\n";
	}
}

init_ofed_rpm_info($CUR_OS_VER);

# ofed specific rpm functions,
# install available user or kernel rpms in list
# returns 1 if ifs-kernel-updates was installed
sub ofed_rpm_install_list($$$@)
{
	my $rpmdir = shift();
	my $osver = shift();	# OS version
	my $skip_kernelib = shift();	# should ifs-kernel-updates be skipped if in package_list
	my(@package_list) = @_;	# package names
	my $ret = 0;

	# user space RPM installs
	foreach my $package ( @package_list )
	{
		if ($ofed_rpm_info{$package}{'Available'} ) {
			if ( "$ofed_rpm_info{$package}{'Mode'}" eq "kernel" ) {
				if ( " $package " =~ / ifs-kernel-updates / ) {
					next if ( $skip_kernelib);
					$ret = 1;
				}
				rpm_install($rpmdir, $osver, $package);
			} else {
				rpm_install($rpmdir, "user", $package);
			}
		}
	}
	return $ret;
}

# verify all rpms in list are installed
sub ofed_rpm_is_installed_list($@)
{
	my $osver = shift();
	my(@package_list) = @_;	# package names

	foreach my $package ( @package_list )
	{
		if ($ofed_rpm_info{$package}{'Available'} ) {
			if ( "$ofed_rpm_info{$package}{'Mode'}" eq "kernel" ) {
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
sub ofed_rpm_exists_list($$@)
{
	my $rpmdir = shift();
	my $mode = shift();	# "user" or kernel rev
	my(@package_list) = @_;	# package names
	my $avail="Available";

	foreach my $package ( @package_list )
	{
		next if ( "$package" eq '' );
		if ($ofed_rpm_info{$package}{$avail} ) {
			if (! rpm_exists($rpmdir, $mode, $package) ) {
				return 0;
			}
		}
	}
	return 1;
}

# returns install prefix for presently installed OFED
sub ofed_get_prefix()
{
	my $prefix = "/usr";	# default
	return "$prefix";
}

# unfortunately OFED mpitests leaves empty directories on uninstall
# this can confuse IFS MPI tools because correct MPI to use
# cannot be identified.  This remove such empty directories for all
# compilers in all possible prefixes for OFED
sub ofed_cleanup_mpitests()
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
sub ofed_rpm_uninstall_not_needed_list($$$$@)
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
			if ( " @ofed_components " =~ / $c /
				 && " $ofed_rpm_info{$package}{'PartOf'} " =~ / $c / ) {
				next RPM;	# its still needed, leave it installed
			}
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
				if ( " @ofed_components " =~ / $c /
					&& " $ofed_rpm_info{$package}{'PartOf'} " =~ / $c / ) {
					# found 1st uninstalled component with package
					if ("$c" ne "$comp") {
						next RPM;	# don't remove til get to $c's uninstall
					} else {
						last;	# exit this loop and uninstall package
					}
				}
			}
		}
		rpm_uninstall($package, "any", "", $verbosity);
	}
}


# resolve filename within $srcdir/$SRPMS_SUBDIR
# and return filename relative to $srcdir
sub ofed_srpm_file($$)
{
	my $srcdir = shift();
	my $globname = shift(); # in $srcdir

	my $result = file_glob("$srcdir/$SRPMS_SUBDIR/$globname");
	$result =~ s|^$srcdir/||;
	return $result;
}

# indicate where OFED built RPMs can be found
sub ofed_rpms_dir()
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
sub get_ofed_rpm_prefix($)
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
sub save_ofed_rpm_prefix($$)
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
	my $mode = shift();	# ""user" or kernel rev
	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};
	my $rpmsdir = ofed_rpms_dir();

	my @rpmlist = split /[[:space:]]+/, $ofed_srpm_info{$srpm}{'Builds'};
	return ( ofed_rpm_exists_list("$rpmsdir", $mode, @rpmlist) );
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

	my @complist = split /[[:space:]]+/, $ofed_srpm_info{$srpm}{'PartOf'};
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
		if ($ofed_rpm_info{$package}{'Available'}) {
			@available_list = ( @available_list, $package );
		}
	}
	if (! scalar(@available_list)) {
		return 0;	# nothing to consider for install
	}
	return ($force
			|| ! ofed_rpm_is_installed_list($osver, @available_list)
			|| ($prompt && GetYesNo("Reinstall @available_list for use during build?", "n")));
}

# move rpms from build tree (srcdir) to install tree (destdir)
sub ofed_move_rpms($$)
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
sub ofed_install_needed_rpms($$$$$@)
{
	my $install_list = shift();	# total that will be installed when done
	my $osver = shift();	# kernel rev
	my $force_rpm = shift();
	my $prompt_rpm = shift();
	my $rpmsdir = shift();
	my @need_install = @_;

	if (need_install_rpm_list($osver, $force_rpm, $prompt_rpm, @need_install)) {
		if (ofed_rpm_install_list("$rpmsdir", $osver, 0, @need_install)) {
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
	my $configure_options = '';	# ofed keeps per srpm, but only initializes here
	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};
	my $SRC_RPM = ofed_srpm_file($srcdir, "$srpm*.src.rpm");

        if ("$srpm" eq "openshmem") {
            $SRC_RPM = ofed_srpm_file($srcdir, "${srpm}_*.src.rpm");
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
        elsif ($parent =~ m/dapl/) {
			# IFS - use rpm_query_param
            my $def_doc_dir = rpm_query_param('_defaultdocdir');
            #my $def_doc_dir = `rpm --eval '%{_defaultdocdir}'`;
            chomp $def_doc_dir;
            $cmd .= " --define '_prefix $prefix'";
            $cmd .= " --define '_exec_prefix $prefix'";
            $cmd .= " --define '_sysconfdir $sysconfdir'";
			# IFS - the srpm name given includes the version for dapl
            $cmd .= " --define '_defaultdocdir $def_doc_dir/$srpm'";
            #$cmd .= " --define '_defaultdocdir $def_doc_dir/$main_packages{$parent}{'name'}-$main_packages{$parent}{'version'}'";
            $cmd .= " --define '_usr $prefix'";
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

		if ($parent eq "librdmacm") {
			# IFS - keep configure_options as a local, ibacm always installed
			#if ( $packages_info{'ibacm'}{'selected'}) {
			if ( $ofed_rpm_info{'ibacm'}{'Available'}) {
				$configure_options .= " --with-ib_acm";
			}
		}

		# IFS - keep configure_options as a local
        if ($configure_options or $OFED_user_configure_options) {
            $cmd .= " --define 'configure_options $configure_options $OFED_user_configure_options'";
        }

		# IFS - use SRC_RPM (computed above) instead of srpmpath_for_distro
#       $cmd .= " $main_packages{$parent}{'srpmpath'}";
		$cmd .= " $SRC_RPM";

	if ("$srpm" eq "hfi1") {
	    $cmd .= " --define 'require_kver 3.12.18-wfr+'";
	}

	if ("$srpm" eq "ib_wfr_lite") {
	    $cmd .= " --define 'require_kver 3.12.18-wfr+'";
	}

	if ("$srpm" eq "gasnet") {
	    $cmd .= " --define '_name gasnet_openmpi_hfi'";
	    $cmd .= " --define '_prefix /usr/shmem/gcc/gasnet-1.24.0-openmpi-hfi'";
	    $cmd .= " --define '_name gasnet_gcc_hfi'";
	    $cmd .= " --define 'spawner mpi'";
	    $cmd .= " --define 'mpi_prefix /usr/mpi/gcc/openmpi-1.10.0-hfi'";
	}

	if ("$srpm" eq "openshmem") {
	    $cmd .= " --define '_name openshmem_gcc_hfi'";
	    $cmd .= " --define '_prefix /usr/shmem/gcc/openshmem-1.0h-hfi'";
	    $cmd .= " --define 'gasnet_prefix /usr/shmem/gcc/gasnet-1.24.0-openmpi-hfi'";
	    $cmd .= " --define 'configargs --with-gasnet-threnv=seq'";
	}

	if ("$srpm" eq "openshmem-test-suite") {
	    $cmd .= " --define '_name openshmem-test-suite_gcc_hfi'";
	    $cmd .= " --define '_prefix /usr/shmem/gcc/openshmem-1.0h-hfi'";
	    $cmd .= " --define 'openshmem_prefix /usr/shmem/gcc/openshmem-1.0h-hfi'";
	}
		return run_build("$srcdir $SRC_RPM $RPM_ARCH", "$srcdir", $cmd, "$resfileop");
	}
	# NOTREACHED
}

# build all OFED components specified in installing_list
# if already built, prompt user for option to build
sub build_ofed($$$$$$)
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
	my $rpmsdir = ofed_rpms_dir();

	my $prefix=$OFED_prefix;
	if ("$prefix" ne get_ofed_rpm_prefix($rpmsdir)) {
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
	my $build_compat_rdma = need_build_srpm("ifs-kernel-updates", "$K_VER", "$K_VER",
		   					$installing_list,
							$force_srpm || $force_kernel_srpm || $OFED_debug,
							$prompt_srpm);

	my $build_ib_bonding = need_build_srpm("ib-bonding", $K_VER, "$K_VER",
		   					$installing_list,
							$force_srpm||$force_kernel_srpm,$prompt_srpm);

	my $need_build = ($build_compat_rdma || $build_ib_bonding);
	my %build_user_srpms = ();

	foreach my $srpm ( @ofed_user_srpms ) {
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

	if ($build_compat_rdma)
	{
		VerbosePrint("check dependencies for ifs-kernel-updates\n");
		if (check_kbuild_dependencies($K_VER, "ifs-kernel-updates")) {
			DebugPrint "ifs-kernel-updates kbuild dependency failure\n";
			$dep_error = 1;
		}
		if (check_rpmbuild_dependencies("ifs-kernel-updates")) {
			DebugPrint "ifs-kernel-updates rpmbuild dependency failure\n";
			$dep_error = 1;
		}
	}
	if ($build_ib_bonding)
	{
		VerbosePrint("check dependencies for ib-bonding\n");
		if (check_kbuild_dependencies($K_VER, "ib-bonding")) {
			DebugPrint "ib-bonding kbuild dependency failure\n";
			$dep_error = 1;
		}
		if (check_rpmbuild_dependencies("ib-bonding")) {
			DebugPrint "ib-bonding rpmbuild dependency failure\n";
			$dep_error = 1;
		}
	}
	foreach my $srpm ( @ofed_user_srpms ) {
		# mpitests is built as part of mvapich, openmpi and mvapich2
		next if ( "$srpm" eq "mpitests" );

		my $build_user = $build_user_srpms{"${srpm}_build_user"};

		next if ( ! $build_user );

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
							@{ $ofed_srpm_info{$srpm}{'BuildPrereq'}})) {
					DebugPrint "$srpm prereqs dependency failure\n";
					$dep_error = 1;
				}
			}
			if ("$srpm" eq "rnfs-utils" ) {
				my @reqs = ('krb5-devel any user');
			if ("$CUR_DISTRO_VENDOR" eq 'redhat' ||
				"$CUR_DISTRO_VENDOR" eq 'fedora') {
					@reqs = ( @reqs, 'krb5-libs any user',
						   		'openldap-devel any user');
					# RHEL6 and later have removed libevent and nfs-utils-lib
					if ("$CUR_VENDOR_VER" eq 'ES5' || "$CUR_VENDOR_VER" eq 'ES4') {
						@reqs = ( @reqs, 'libevent-devel any user',
						   		'nfs-utils-lib-devel any user');
					}
				} else {
					@reqs = ( @reqs, 'krb5 any user',
							'openldap2-devel any user',
							'cyrus-sasl-devel any user');
				}
				if ("$CUR_DISTRO_VENDOR" eq 'SuSE') {
					if ($K_VER =~ m/2\.6\.2[6-7]/ ) {
						@reqs = ( @reqs, 'libblkid-devel any user');
					} else {
						@reqs = ( @reqs, 'e2fsprogs-devel any user');
					}
					if ("$CUR_VENDOR_VER" eq 'ES11') {
						@reqs = ( @reqs, 'libevent-devel any user',
						   	'nfsidmap-devel any user',
						   	'libopenssl-devel any user');
					}
				}
				if (rpm_check_build_os_prereqs("any", "$srpm for $ComponentInfo{'ofed_nfsrdma'}{'Name'}", (@reqs))) {
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

	if ("$prefix" ne get_ofed_rpm_prefix($rpmsdir)) {
		system("rm -rf $rpmsdir");	# get rid of stuff with old prefix
		save_ofed_rpm_prefix($rpmsdir, $prefix);
	}

	# use a different directory for BUILD_ROOT to limit conflict with OFED
	if ("$build_temp" eq "" ) {
		$build_temp = "/var/tmp/IntelOPA-OFED";
	}
	my $BUILD_ROOT="$build_temp/build";
	my $RPM_DIR="$build_temp/OFEDRPMS";
	my $mandir = rpm_query_param("_mandir");
	my $resfileop = "replace";	# replace for 1st build, append for rest

	system("rm -rf ${build_temp}");

	# use a different directory for BUILD_ROOT to limit conflict with OFED
	if (0 != system("mkdir -p $BUILD_ROOT $RPM_DIR/BUILD $RPM_DIR/RPMS $RPM_DIR/SOURCES $RPM_DIR/SPECS $RPM_DIR/SRPMS")) {
		NormalPrint "ERROR - mkdir -p $BUILD_ROOT $RPM_DIR/BUILD $RPM_DIR/RPMS $RPM_DIR/SOURCES $RPM_DIR/SPECS $RPM_DIR/SRPMS FAILED\n";
		return 1;	# failure
	}

	my $rpm_os_version=rpm_tr_os_version($K_VER);

	# OFED has all the ULPs in a single ifs-kernel-updates RPM.  We build that
	# RPM here from the ifs-kernel-updates SRPM with all ULPs included.
	# Later during install we remove ULPs not desired after installing
	# the ifs-kernel-updates RPM
	if ($build_compat_rdma)
	{
		my $OFA_KERNEL_SRC_RPM=ofed_srpm_file($srcdir,"ifs-kernel-updates*.src.rpm");
		my $K_SRC = "/lib/modules/$K_VER/build";
		my $configure_options_kernel;
		my $cok_macro;
		$configure_options_kernel = get_build_options($K_VER, %ofed_kernel_ib_options);
		if ( $OFED_debug ) {
			# TBD --with-memtrack
			#$configure_options_kernel .= " --with-memtrack";
		}

		VerbosePrint("OS specific kernel configure options: '$configure_options_kernel'\n");

		if ($configure_options_kernel != "") {
			$cok_macro=" --define 'configure_options ${configure_options_kernel} $OFED_kernel_configure_options'";
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
				.		" --define '_release $rpm_os_version'"
				.		" ${OFA_KERNEL_SRC_RPM}",
				"$resfileop"
				)) {
			return 1;	# failure
		}
		$must_force_rpm=1;
		$resfileop = "append";
		ofed_move_rpms("$RPM_DIR/$RPMS_SUBDIR", "$rpmsdir");
	}
	@need_install = ( @need_install, split /[[:space:]]+/, $ofed_srpm_info{'ifs-kernel-updates'}{'PostReq'});

	if ($build_ib_bonding)
	{
		my $IB_BONDING_SRC_RPM=ofed_srpm_file($srcdir,"ib-bonding*.src.rpm");

		# OFED_kernel_configure_options not applicable to ib-bonding
		if (0 != run_build("$srcdir $IB_BONDING_SRC_RPM $RPM_KERNEL_ARCH $K_VER", "$srcdir",
				 "rpmbuild --rebuild --define '_topdir ${RPM_DIR}'"
        		.		" --target $RPM_KERNEL_ARCH"
				.		" --define '_prefix ${prefix}'"
				.		" --buildroot '${BUILD_ROOT}'"
				.		" --define 'build_root ${BUILD_ROOT}'"
				.		" --define 'KVERSION ${K_VER}'"
				.		" --define '_release ${rpm_os_version}'"
            	.		" --define '__arch_install_post %{nil}'"
				.		" ${IB_BONDING_SRC_RPM}",
				"$resfileop"
				)) {
			return 1;	# failure
		}
		$resfileop = "append";
		ofed_move_rpms("$RPM_DIR/$RPMS_SUBDIR", "$rpmsdir");
		$must_force_rpm=1;
	}
	@need_install = ( @need_install, split /[[:space:]]+/, $ofed_srpm_info{'ib-bonding'}{'PostReq'});

	foreach my $srpm ( @ofed_user_srpms ) {
		VerbosePrint("process $srpm\n");

		# mpitests is built as part of mvapich, openmpi and mvapich2
		next if ( "$srpm" eq "mpitests" );

		my $build_user = $build_user_srpms{"${srpm}_build_user"};
			if ($build_user) {
				# load rpms which are were PostReqs from previous srpm builds
				ofed_install_needed_rpms($install_list, $K_VER, $force_rpm||$must_force_rpm, $prompt_rpm, $rpmsdir, @need_install);
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
				} elsif ("$srpm" eq "openmpi" ) {
					if (0 != run_build("openmpi_gcc and mpitests_openmpi_gcc", "$srcdir", "STACK_PREFIX='$prefix' BUILD_DIR='$build_temp' MPICH_PREFIX= CONFIG_OPTIONS='$OFED_user_configure_options' INSTALL_ROOT='$ROOT' ./do_openmpi_build -d -i gcc", $resfileop)) {
						return 1;	# failure
					}
					# build already installed openmpi_gcc
					@installed = ( @installed, split /[[:space:]]+/, 'openmpi_gcc');
					$resfileop = "append";
					$must_force_rpm=1;

					ofed_move_rpms("$RPM_DIR/$RPMS_SUBDIR", "$rpmsdir");

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
					@need_install = ( @need_install, split /[[:space:]]+/, $ofed_srpm_info{$srpm}{'PostReq'});
					$must_force_rpm=1;
				}
				ofed_move_rpms("$RPM_DIR/$RPMS_SUBDIR", "$rpmsdir");
			} else {
				@need_install = ( @need_install, split /[[:space:]]+/, $ofed_srpm_info{$srpm}{'PostReq'});
			}
		}

	# get rid of rpms we installed to enable builds but are not desired to stay
	# eg. uninstall rpms which were installed but are not part of install_list
	ofed_rpm_uninstall_not_needed_list($install_list, "", "", "verbose", @installed);

	if (! $debug) {
		system("rm -rf ${build_temp}");
	} else {
		LogPrint "Build remnants left in $BUILD_ROOT and $RPM_DIR\n";
	}

	return 0;	# success
}

# forward declarations
sub installed_ofed_opa_stack();
sub uninstall_old_stacks($);

# track if install_kernel_ib function was used so we only install
# ifs-kernel-updates once in a given "Perform" of install menu
my $install_kernel_ib_was_run = 0;

# return 0 on success, != 0 otherwise
sub uninstall_old_ofed_rpms($$$)
{
	my $mode = shift();	# "user" or kernel rev
						# "any"- checks if any variation of package is installed
	my $verbosity = shift();
	my $message = shift();

	my $ret = 0;	# assume success
	my @packages = ();

	if ("$message" eq "" ) {
		$message = "previous OFA";
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

	# uninstall all present version OFED rpms, just to be safe
	foreach my $i ( reverse(@ofed_components) ) {
		@packages = (@packages, @{ $ofed_comp_info{$i}{'DebugRpms'}});
		@packages = (@packages, @{ $ofed_comp_info{$i}{'UserRpms'}});
		@packages = (@packages, @{ $ofed_comp_info{$i}{'KernelRpms'}});
	}
	# uninstall RPMs from older versions of OFED
	@packages = (@packages, @prev_release_rpms);
	@packages = (@packages, @redhat_rpms);
	@packages = (@packages, @suse_rpms);
	# other potential RPMs for ofed
	#my @other_ofed_rpms = `rpm -qa 2> /dev/null | grep ofed`;
	#@packages = (@packages, @other_ofed_rpms);

	# workaround LAM and other MPIs usng mpi-selector
	# we uninstall mpi-selector separately and ignore failures for its uninstall
	my @filtered_packages = ();
	my @rest_packages = ();
	foreach my $i ( @packages ) {
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
		} elsif ( ("$i" eq "ib-bonding" || "$i" eq "ib-bonding-debuginfo")
					&& ! $ofed_rpm_info{$i}{'Available'}) {
			# skip, might be included with distro
		} else {
			@filtered_packages = (@filtered_packages, "$i");
		}
	}

	$ret ||= rpm_uninstall_all_list($mode, $verbosity, @filtered_packages);

	# ignore errors uninstalling mpi-selector
	if (scalar(@rest_packages) != 0) {
		if (rpm_uninstall_all_list($mode, $verbosity, @rest_packages) && ! $ret) {
			NormalPrint "The previous errors can be ignored\n";
		}
	}

	ofed_cleanup_mpitests();

	if ( $ret ) {
		NormalPrint "Unable to uninstall $message RPMs\n";
	}
	return $ret;
}

# The QuickSilver version of SDP can leave sunrpc in a patched state
# in which case nfs will not load because it depends on QuickSilver SDP module
sub cleanup_iba_sdp()
{
	# only care about 2.6 kernels (.ko modules, not .o)
	if ( -e "$ROOT/lib/modules/$CUR_OS_VER/kernel/net/sunrpc/sunrpc.ko_orig" ) {
		my $res=system("cmp -s $ROOT/lib/modules/$CUR_OS_VER/kernel/net/sunrpc/sunrpc.ko_sdp $ROOT/lib/modules/$CUR_OS_VER/kernel/net/sunrpc/sunrpc.ko");
		if ( $res == 0) {
			NormalPrint "Undoing QuickSilver SDP NFS hooks\n";
			# Quicksilver sdp version is the active version, restore orig
			my $cmd="cp $ROOT/lib/modules/$CUR_OS_VER/kernel/net/sunrpc/sunrpc.ko_orig $ROOT/lib/modules/$CUR_OS_VER/kernel/net/sunrpc/sunrpc.ko";
			LogPrint "$cmd\n";
			if (0 != system("$cmd")) {
				NormalPrint "ERROR - Failed to $cmd\n";
			}
		}
	}
}

# remove any old stacks or old versions of the stack
# this is necessary before doing builds to ensure we don't use old dependent
# rpms
sub uninstall_prev_versions()
{
	cleanup_iba_sdp();
	if (! installed_ofed_opa_stack) {
		if (0 != uninstall_old_stacks(1)) {
			return 1;
		}
	} elsif (! comp_is_uptodate('opa_stack')) { # all ofed_comp same version
		if (0 != uninstall_old_ofed_rpms("any", "silent", "previous OFED")) {
			return 1;
		}
	}
	return 0;
}

sub media_version_ofed()
{
	# all OFED components at same version as opa_stack
	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};
	return `cat "$srcdir/Version"`;
}

sub ofed_save_autostart()
{
	foreach my $comp ( @ofed_components ) {
  		if ($ComponentInfo{$comp}{'HasStart'}
			&& $ofed_comp_info{$comp}{'StartupScript'} ne "") { 
			$ofed_autostart_save{$comp} = comp_IsAutostart2($comp);
		} else {
			$ofed_autostart_save{$comp} = 0;
		}
	}
}

sub ofed_restore_autostart($)
{
	my $comp = shift();

	if ( $ofed_autostart_save{$comp} ) {
		comp_enable_autostart2($comp, 1);
	} else {
  		if ($ComponentInfo{$comp}{'HasStart'}
			&& $ofed_comp_info{$comp}{'StartupScript'} ne "") { 
			comp_disable_autostart2($comp, 1);
		}
	}
}

# makes sure needed OFED components are already built, builts/rebuilds as needed
# called for every ofed component's preinstall, noop for all but
# first OFED component in installing_list
# also available for use by srp to facilitate them getting
# built within ofed context
sub preinstall_ofed($$$)
{
	my $comp = shift();			# calling component
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	# make sure flag cleared so will install this if part of an installed comp
	$install_kernel_ib_was_run = 0;

	# ignore non-ofed components at start of installing_list
	my @installing = split /[[:space:]]+/, $installing_list;
	while (scalar(@installing) != 0
			&& ("$installing[0]" eq ""
			 	|| " @ofed_components " !~ / $installing[0] /)) {
		shift @installing;
	}
	# now, only do work if $comp is the 1st ofed component in installing_list
	if ("$comp" eq "$installing[0]") {
		ofed_save_autostart();
		init_ofed_rpm_info($CUR_OS_VER);

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
		my $version=media_version_ofed();
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
			my $rc = build_ofed("$install_list", "$installing_list", "$CUR_OS_VER",0,"",$OFED_force_rebuild);
			$ROOT=$save_ROOT;
			return $rc;
		} else {
			return build_ofed("$install_list", "$installing_list", "$CUR_OS_VER",0,"",$OFED_force_rebuild);
		}
	} else {
		return 0;
	}
}

# wrapper for installed_driver to handle the fact that OFED drivers used to
# be installed under updates/kernel and now are simply installed under updates
# So we check both places so that upgrade installs can properly detect drivers
# which are in the old location
sub installed_ofed_driver($$$)
{
	my $WhichDriver = shift();
	my $driver_subdir = shift();
	my $subdir = shift();

	return installed_driver($WhichDriver, "$driver_subdir/$subdir")
			|| installed_driver($WhichDriver, "$driver_subdir/kernel/$subdir");
}

# ==========================================================================
# OFED generic installation routines

# since most of OFED components are simply data driven by Rpm lists in
# ofed_comp_info, we can implement these support functions which do
# most of the real work for install and uninstall of components

# OFED has a single start script but controls which ULPs are loaded via
# entries in $OFED_CONFIG (openib.conf)
# change all StartupParams for given ofed component to $newvalue
sub ofed_comp_change_openib_conf_param($$)
{
	my $comp=shift();
	my $newvalue=shift();

	VerbosePrint("edit $ROOT/$OFED_CONFIG: $comp StartUp set to '$newvalue'\n");
	foreach my $p ( @{ $ofed_comp_info{$comp}{'StartupParams'} } ) {
		change_openib_conf_param($p, $newvalue);
	}
}

# generic functions to handle autostart needs for ofed components with
# more complex openib.conf based startup needs.  These assume opa_stack handles
# the actual startup script.  Hence these focus on the openib.conf parameters
# determine if the given capability is configured for Autostart at boot
sub IsAutostart_ofed_comp2($)
{
	my $comp = shift();	# component to check
	my $WhichStartup = $ofed_comp_info{$comp}{'StartupScript'};
	my $ret = IsAutostart($WhichStartup);	# just to be safe, test this too

	# to be true, all parameters must be yes
	foreach my $p ( @{ $ofed_comp_info{$comp}{'StartupParams'} } ) {
			$ret &= ( read_openib_conf_param($p, "") eq "yes");
	}
	return $ret;
}
sub autostart_desc_ofed_comp($)
{
	my $comp = shift();	# component to describe
	my $WhichStartup = $ofed_comp_info{$comp}{'StartupScript'};
	return "$ComponentInfo{$comp}{'Name'} ($WhichStartup)";
}
# enable autostart for the given capability
sub enable_autostart_ofed_comp2($)
{
	my $comp = shift();	# component to enable
	#my $WhichStartup = $ofed_comp_info{$comp}{'StartupScript'};

	#opa_stack handles this: enable_autostart($WhichStartup);
	ofed_comp_change_openib_conf_param($comp, "yes");
}
# disable autostart for the given capability
sub disable_autostart_ofed_comp2($)
{
	my $comp = shift();	# component to disable
	#my $WhichStartup = $ofed_comp_info{$comp}{'StartupScript'};

	# Note: as a side effect this ULP will also not be manually started
	# when openibd is manually run
	#opa_stack handles this: disable_autostart($WhichStartup);
	ofed_comp_change_openib_conf_param($comp, "no");
}

# OFED has all the ULPs in a single RPM.  This function removes the
# drivers from ifs-kernel-updates specific to the given ofed_component
# this is a hack, but its better than uninstall and reinstall ifs-kernel-updates
# every time a ULP is added/removed
sub remove_ofed_kernel_ib_drivers($$)
{
	my $comp = shift();	# component to remove
	my $verbosity = shift();
	# cheat on driver_subdir so ofed_components can have some stuff not
	# in @Components
	# we know driver_subdir is same for all ofed components in ifs-kernel-updates
	my $driver_subdir=$ComponentInfo{'opa_stack'}{'DriverSubdir'};

	my $i;
	my @list = split /[[:space:]]+/, $ofed_comp_info{$comp}{'Drivers'};
	for ($i=0; $i < scalar(@list); $i+=2)
	{
		my $driver=$list[$i];
		my $subdir=$list[$i+1];
		remove_driver("$driver", "$driver_subdir/$subdir", $verbosity);
		remove_driver_dirs("$driver_subdir/$subdir");
	}
}

sub print_install_banner_ofed_comp($)
{
	my $comp = shift();

	my $version=media_version_ofed();
	chomp $version;
	printf("Installing $ComponentInfo{$comp}{'Name'} $version $DBG_FREE...\n");
	# all OFED components at same version as opa_stack
	LogPrint "Installing $ComponentInfo{$comp}{'Name'} $version $DBG_FREE for $CUR_OS_VER\n";
}

# helper to determine if we need to reinstall due to parameter change
sub need_reinstall_ofed_comp($$$)
{
	my $comp = shift();
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	if (get_ofed_rpm_prefix(ofed_rpms_dir()) ne "$OFED_prefix" ) {
		return "all";
	} elsif (! comp_is_uptodate('opa_stack')) { # all ofed_comp same version
		# on upgrade force reinstall to recover from uninstall of old rpms
		return "all";
	} else {
		return "no";
	}
}

# helper which does most of the work for installing rpms and drivers
# for an OFED component
# installs ifs-kernel-updates drivers, KernelRpms and UserRpms
# caller must handle any non-RPM files
sub install_ofed_comp($$)
{
	my $comp = shift();
	my $install_list = shift();	# total that will be installed when done

	# special handling for ifs-kernel-updates
	if ($ComponentInfo{$comp}{'DriverSubdir'} ne "" ) {
		install_kernel_ib(ofed_rpms_dir(), $install_list);
	}
	# skip ifs-kernel-updates if in KernelRpms/UserRpms, already handled above
	ofed_rpm_install_list(ofed_rpms_dir(), $CUR_OS_VER, 1,
							( @{ $ofed_comp_info{$comp}{'KernelRpms'}},
							@{ $ofed_comp_info{$comp}{'UserRpms'}}) );
	# DebugRpms are installed as part of 'ofed_debug' component

}

sub print_uninstall_banner_ofed_comp($)
{
	my $comp = shift();

	NormalPrint("Uninstalling $ComponentInfo{$comp}{'Name'}...\n");
}

# uninstall all rpms associated with an OFED component
sub uninstall_ofed_comp_rpms($$$$)
{
	my $comp = shift();
	my $install_list = shift();
	my $uninstalling_list = shift();
	my $verbosity = shift();

	if ( "$comp" eq "ofed_ib_bonding" && ! $ofed_rpm_info{'ib-bonding'}{'Available'}) {
		# special case - do not touch distro ib-bonding
		return;
	}
	# debuginfo never in >1 component, so do explicit uninstall since
	# have an odd PartOf relationship which confuses uninstall_not_needed_list
	rpm_uninstall_list("any", $verbosity,
					 @{ $ofed_comp_info{$comp}{'DebugRpms'}});
	ofed_rpm_uninstall_not_needed_list($install_list, $uninstalling_list, $comp,
				 	$verbosity, @{ $ofed_comp_info{$comp}{'UserRpms'}});
	ofed_rpm_uninstall_not_needed_list($install_list, $uninstalling_list, $comp,
				 	$verbosity, @{ $ofed_comp_info{$comp}{'KernelRpms'}});
}

# helper which does most of the work for uninstalling rpms and drivers
# for an OFED component
# caller must handle any non-RPM files
sub uninstall_ofed_comp($$$$)
{
	my $comp = shift();
	my $install_list = shift();
	my $uninstalling_list = shift();
	my $verbosity = shift();

	uninstall_ofed_comp_rpms($comp, $install_list, $uninstalling_list, $verbosity);
	remove_ofed_kernel_ib_drivers($comp, $verbosity);
}

# remove ifs-kernel-updates drivers for components which will not be installed
sub remove_unneeded_kernel_ib_drivers($)
{
	my $install_list = shift();	# total that will be installed when done
	foreach my $c ( @ofed_components )
	{
		if ($install_list !~ / $c /) {
			remove_ofed_kernel_ib_drivers($c, "verbose");
		}
	}
}

# OFED has all the ULPs in a single RPM.  This function installs that
# RPM and removes the undesired ULP drivers
sub install_kernel_ib($$)
{
	my $rpmdir = shift();
	my $install_list = shift();	# total that will be installed when done

	my $driver_subdir=$ComponentInfo{'opa_stack'}{'DriverSubdir'};	# same for all ofed components

	if ( $install_kernel_ib_was_run) {
		return;
	}

	rpm_install("$rpmdir", $CUR_OS_VER, "ifs-kernel-updates");
	remove_unneeded_kernel_ib_drivers($install_list);

	# openib.conf values not directly associated with driver startup are left
	# untouched ofed rpm install will keep existing value

	$install_kernel_ib_was_run = 1;
}

# ==========================================================================
# OFED opa_stack installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_opa_stack()
{
	# opa_stack is tricky, there are multiple parameters.  We just test
	# the things we control here, if user has edited openib.conf they
	# could end up with startup still disabled by having disabled all
	# the individual HCA drivers
	return IsAutostart_ofed_comp2('opa_stack') || IsAutostart("iba");
}
sub autostart_desc_opa_stack()
{
	return autostart_desc_ofed_comp('opa_stack');
}
# enable autostart for the given capability
sub enable_autostart2_opa_stack()
{
	enable_autostart("openibd");
	ofed_comp_change_openib_conf_param('opa_stack', "yes");
}
# disable autostart for the given capability
sub disable_autostart2_opa_stack()
{
	disable_autostart("openibd");
	ofed_comp_change_openib_conf_param('opa_stack', "no");
}

sub start_opa_stack()
{
	my $driver_subdir=$ComponentInfo{'opa_stack'}{'DriverSubdir'};
	start_driver($ComponentInfo{'opa_stack'}{'Name'}, "ib_core", "$driver_subdir/drivers/infiniband/core", "openibd");
}

sub stop_opa_stack()
{
	stop_driver($ComponentInfo{'opa_stack'}{'Name'}, "ib_core", "openibd");
}

sub available_opa_stack()
{
	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};
# TBD better checks for available?
# check file_glob("$srcdir/SRPMS/ifs-kernel-updates*.src.rpm") ne ""
#			|| rpm_exists($rpmsdir, $CUR_OS_VER, "ifs-kernel-updates")
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_ofed_opa_stack()
{
	my $driver_subdir=$ComponentInfo{'opa_stack'}{'DriverSubdir'};
	return (rpm_is_installed("libibverbs", "user")
			&& -e "$ROOT$BASE_DIR/version_ofed"
			&& rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER)
	# MWHEINZ - commented out because ifs-kernel-updates does not follow
	# the old ofa_kernel installation model.
	#		&& installed_ofed_driver("ib_core", "$driver_subdir", "drivers/infiniband/core")
			 );
}

sub installed_opa_stack()
{
	return (installed_ofed_opa_stack);
}

# only called if installed_opa_stack is true
sub installed_version_opa_stack()
{
	if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
		return `cat $ROOT$BASE_DIR/version_ofed`;
	} else {
		return 'NONE';
	}
}

# only called if available_opa_stack is true
sub media_version_opa_stack()
{
	return media_version_ofed();
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

# uninstall 3rd party and older IB stacks
# only called when opa_stack is not installed, hence it can also safely
# uninstall files/rpms which may overlap with the opa_stack
# return 0 on success, != 0 on failure
sub	uninstall_old_stacks($)
{
	my $prompt = shift();
	my $ret = 0;

	if ( $prompt) {
		NormalPrint "About to Uninstall previous IB Software Installations...\n";
		HitKeyCont;
	}

	# FF for OFED requires oftools
	# hence we can conclude if FF for OFED is installed by testing
	# for opainfo (which is unique to oftools for OFED)
	# this way if FF for OFED is already installed, we
	# will not run opaconfig -u
	# also check our name so we don't recursively call ourselves
	# However if FF for QS or QS is installed, we want to run uninstall
	if (-e "$ROOT/sbin/opaconfig"
		&& ! -e "$ROOT$BASE_DIR/version_ofed"
		&& ! -e "$ROOT/usr/sbin/opainfo"
		&& my_basename($0) ne "opaconfig"
		) {
		$ret |= run_uninstall("QLogic/QuickSilver/SilverStorm IB Stack", "$ROOT/sbin/opaconfig", "-u -r '$ROOT'");
	}

	# uninstall Mellanox OFED
	$ret |= run_uninstall("Mellanox OFED IB Stack", "/sbin/mlnx_en_uninstall.sh", "" );
	# uninstall modern OFED
	my $ofed_uninstall=`chroot /$ROOT which ofed_uninstall.sh 2>/dev/null`;
	$ret |= run_uninstall("OFED IB Stack", "$ofed_uninstall", "");
	my $prefix = ofed_get_prefix();
	$ret |= run_uninstall("OFED IB Stack", "$prefix/sbin/ofed_uninstall.sh", "" );
	$ret |= run_uninstall("OFED IB Stack", "$prefix/uninstall_gen2.sh", "" );
	$ret |= run_uninstall("OFED IB Stack", "$prefix/uninstall.sh", "" );
	ofed_cleanup_mpitests();

	# uninstall TopSpin/Cisco
	if (rpm_uninstall_matches("TopSpin/Cisco IB Stack", "topspin-ib", "")) {
		NormalPrint "Unable to uninstall TopSpin/Cisco IB Stack\n";
		$ret = 1;
	}

	# uninstall Voltaire
	if (rpm_uninstall_matches("Voltaire IB Stack", "Voltaire", "4.0.0_5")) {
		NormalPrint "Unable to uninstall Voltaire IB Stack\n";
		$ret = 1;
	}

	# uninstall Mellanox
	if ("$ENV{MTHOME}" ne "") {
		$ret |= run_uninstall("Mellanox IB Stack", "$ENV{MTHOME}/uninstall.sh", "");
	}
	$ret |= run_uninstall("Mellanox IB Stack", "/usr/mellanox/uninstall.sh", "");
	$ret |= run_uninstall("Mellanox EN driver", "/sbin/mlnx_en_uninstall.sh", "" );
	# uninstall Mellanox mtnic
	if ( -d "$ROOT/lib/modules/$CUR_OS_VER/kernel/drivers/net/mtnic" ) {
		NormalPrint "\nUninstalling mtnic driver: chroot /$ROOT /sbin/rmmod mtnic >/dev/null 2>&1; chroot /$ROOT rm -rf /lib/modules/*/kernel/drivers/net/mtnic\n";
		system("chroot /$ROOT /sbin/rmmod mtnic >/dev/null 2>&1; chroot /$ROOT rm -rf /lib/modules/*/kernel/drivers/net/mtnic");
	}

	# workaround a %preun problem in RHEL 5 which included OFED 1.1
	rpm_uninstall_matches("OFED 1.1 iscsi","open-iscsi-2.0-754","",
   				"--nodeps --notriggers --noscripts");
	rpm_uninstall_matches("OFED 1.1","openib-1.1","",
   				"--nodeps --notriggers --noscripts");

	# workaround a %preun problem in RHEL 5.1 which included OFED 1.2
	rpm_uninstall_matches("OFED 1.2","openib-1.2","",
   				"--nodeps --notriggers --noscripts");

	# workaround a %preun problem in RHEL 5.3 which included OFED 1.3
	rpm_uninstall_matches("OFED 1.3","iscsi-target-utils-0.1-20080828","",
   				"--nodeps --notriggers --noscripts");
	rpm_uninstall_matches("OFED 1.3","tgt-0.1-20080828","",
   				"--nodeps --notriggers --noscripts");

	# workaround a %preun problem in SLES11 which included OFED 1.4
	rpm_uninstall_matches("OFED 1.4","opensm-3.2.3_20081117_b70e2d2-1.10","",
   				"--nodeps --notriggers --noscripts");
	rpm_uninstall_matches("OFED 1.4","ofed-1.4.0-11.12","",
   				"--nodeps --notriggers --noscripts");

	$ret |= uninstall_old_ofed_rpms("any", "silent", "previous OFED");

	my $old_ofed = file_glob("$ROOT/usr/local/ofed*");
	if ("$old_ofed" ne "") {
		NormalPrint "\nUninstalling previous OFED code: rm -rf $ROOT/usr/local/ofed*\n";
		system("rm -rf $ROOT/usr/local/ofed* >/dev/null 2>/dev/null");
	}

	# some other possible stray files which could be left around
	system("rm -rf $ROOT/usr/include/infiniband/complib");
	system("rm -rf $ROOT/usr/include/infiniband/vendor");
	system("rm -rf $ROOT/usr/include/infiniband/opensm");
	system("rm -rf $ROOT/usr/include/infiniband/iba");
	system("rm -rf $ROOT/usr/libosmcomp*");
	system("rm -rf $ROOT/usr/libosmvendor*");
	system("rm -rf $ROOT/usr/libopensm*");
	system("rm -rf $ROOT/usr/bin/opensm");
	system("rm -rf $ROOT/usr/bin/osmtest");
	system("rm -rf $ROOT/usr/local/include/infiniband/complib");
	system("rm -rf $ROOT/usr/local/include/infiniband/vendor");
	system("rm -rf $ROOT/usr/local/include/infiniband/opensm");
	system("rm -rf $ROOT/usr/local/include/infiniband/iba");
	system("rm -rf $ROOT/usr/local/libosmcomp*");
	system("rm -rf $ROOT/usr/local/libosmvendor*");
	system("rm -rf $ROOT/usr/local/libopensm*");
	system("rm -rf $ROOT/usr/local/bin/opensm");
	system("rm -rf $ROOT/usr/local/bin/osmtest");

	return $ret;
}

# return 0 on success, !=0 on failure
sub build_opa_stack($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild

	init_ofed_rpm_info($osver);

	# Before we do any builds make sure old stacks removed so we don't
	# build against the wrong version of dependent rpms
	if (0 != uninstall_prev_versions()) {
		return 1;
	}

	if (ROOT_is_set()) {
		# build in current image
		my $save_ROOT=$ROOT;
		$ROOT='/';
		my $rc =  build_ofed("@Components", "@Components", $osver, $debug,$build_temp,$force);
		$ROOT=$save_ROOT;
		return $rc;
	} else {
		return build_ofed("@Components", "@Components", $osver, $debug,$build_temp,$force);
	}
}

sub need_reinstall_opa_stack($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_ofed_comp('opa_stack', $install_list, $installing_list));
}

sub check_os_prereqs_opa_stack
{
 	return rpm_check_os_prereqs("opa_stack", "any", (
				'pciutils', 'libstdc++ any user'
   				));
}

sub preinstall_opa_stack($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("opa_stack", $install_list, $installing_list);
}

sub install_opa_stack($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	my $srcdir=$ComponentInfo{'opa_stack'}{'SrcDir'};

	print_install_banner_ofed_comp('opa_stack');

	#override the udev permissions.
	install_udev_permissions("$srcdir/config");
        # set environment variable for RPM to configure linmits_conf
	# edit_limitsconf("$srcdir/config");
	setup_env("OPA_LIMITS_CONF", 1);
        # We also need to install driver, so setting up envirnment
        # to install driver for this component. actual install is done by rpm
	setup_env("OPA_INSTALL_DRIVER", 1);

	# Check $BASE_DIR directory ...exist 
	check_config_dirs();
	check_dir("/usr/lib/opa");

	copy_systool_file("$srcdir/comp.pl", "/usr/lib/opa/.comp_ofed.pl");

	install_ofed_comp('opa_stack', $install_list);

	# in some recovery situations OFED doesn't properly restore openib.conf
	# this makes sure the SMA NodeDesc is properly set so the hostnmae is used
	change_openib_conf_param('NODE_DESC', '$(hostname -s)');
	prompt_openib_conf_param('RENICE_IB_MAD', 'OFED SMI/GSI renice', "y", 'OPA_RENICE_IB_MAD');
	# QIB driver has replaced IPATH, make sure IPATH disabled
	change_openib_conf_param('IPATH_LOAD', 'no');
	# MWHEINZ FIXME - disabled because the qib driver is loaded 
	# automagically by the 3.9.2 kernel.
	#if (!build_option_is_allowed($CUR_OS_VER, "--with-qib-mod", %ofed_kernel_ib_options)) {
	#	change_openib_conf_param('QIB_LOAD', 'no');
	#} else {
	#	change_openib_conf_param('QIB_LOAD', 'yes');
	#}
	# MWHEINZ FIXME - it appears that we are dropping kcopy support.
	#if (!build_option_is_allowed($CUR_OS_VER, "--with-kcopy-mod", %ofed_kernel_ib_options)) {
	#	change_openib_conf_param('KCOPY_LOAD', 'no');
	#} else {
	#	change_openib_conf_param('KCOPY_LOAD', 'yes');
	#}

	copy_data_file("$srcdir/Version", "$BASE_DIR/version_ofed");

	# prevent distro's open IB from loading
	#add_blacklist("ib_mthca");
	#add_blacklist("ib_ipath");
	disable_distro_ofed();
	need_reboot();
	$ComponentWasInstalled{'opa_stack'}=1;
}

sub postinstall_opa_stack($$)
{
	my $old_conf = 0;	# do we have an existing conf file
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	if ( -e "$ROOT/$OFED_CONFIG" ) {
		if (0 == system("cp $ROOT/$OFED_CONFIG $ROOT/$OFED_CONFIG-save")) {
			$old_conf=1;
		}
	}

	# adjust openib.conf autostart settings
	foreach my $c ( @ofed_components )
	{
		if ($install_list !~ / $c /) {
			# disable autostart of uninstalled components
			# no need to disable openibd, since being in this function implies
			# opa_stack is at least installed
			ofed_comp_change_openib_conf_param($c, "no");
		} else {
			# retain previous setting for components being installed
			# set to no if initial install
			# it seems that ifs-kernel-updates rpm will do this for us,
			# repeat just to be safe
			foreach my $p ( @{ $ofed_comp_info{$c}{'StartupParams'} } ) {
				my $old_value = "";
				if ( $old_conf ) {
					$old_value = read_openib_conf_param($p, "$ROOT/$OFED_CONFIG-save");
				}
				if ( "$old_value" eq "" ) {
					$old_value = "no";
				}
				change_openib_conf_param($p, $old_value);
			}
		}
	}

	ofed_restore_autostart('opa_stack');
}

sub uninstall_opa_stack($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	my $driver_subdir=$ComponentInfo{'opa_stack'}{'DriverSubdir'};
	print_uninstall_banner_ofed_comp('opa_stack');
	stop_opa_stack;
	remove_blacklist("ib_qib");
	remove_blacklist("ib_wfr_lite");

	# allow open IB to load
	#remove_blacklist("ib_mthca");
	#remove_blacklist("ib_ipath");

	# ofed rpm -e will keep an rpmsave copy of openib.conf for us
	uninstall_ofed_comp('opa_stack', $install_list, $uninstalling_list, 'verbose');
	remove_driver_dirs($driver_subdir);
	#remove_modules_conf;
	remove_limits_conf;

	remove_udev_permissions;

	system("rm -rf $ROOT$BASE_DIR/version_ofed");
	system("rm -rf $ROOT/usr/lib/opa/.comp_ofed.pl");
	system "rmdir $ROOT/usr/lib/opa 2>/dev/null";	# remove only if empty
	system "rmdir $ROOT$BASE_DIR 2>/dev/null";	# remove only if empty
	system "rmdir $ROOT$OPA_CONFIG_DIR 2>/dev/null";	# remove only if empty
	if (0 != uninstall_old_stacks(0)) {
		Abort "Unable to uninstall old IB stacks\n";
	}
	need_reboot();
	$ComponentWasInstalled{'opa_stack'}=0;
}
# ==========================================================================
# intel_hfi installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_intel_hfi()
{
    return IsAutostart_ofed_comp2('intel_hfi');
}
sub autostart_desc_intel_hfi()
{
    return autostart_desc_ofed_comp('intel_hfi');
}
# enable autostart for the given capability
sub enable_autostart2_intel_hfi()
{
    enable_autostart_ofed_comp2('intel_hfi');
}
# disable autostart for the given capability
sub disable_autostart2_intel_hfi()
{
    disable_autostart_ofed_comp2('intel_hfi');
}

sub available_intel_hfi()
{
    my $srcdir=$ComponentInfo{'intel_hfi'}{'SrcDir'};
        return ( (-d "$srcdir/SRPMS" || -d "$srcdir/RPMS" )
		 && build_option_is_allowed($CUR_OS_VER, "", %ofed_kernel_ib_options));
}

sub installed_intel_hfi()
{
    my $driver_subdir=$ComponentInfo{'intel_hfi'}{'DriverSubdir'};
        return (rpm_is_installed("libhfiverbs", "user")
                        && -e "$ROOT$BASE_DIR/version_ofed"
                        && rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER)
        # JFLECK: Needs to be adjusted for hfi elements

        # MWHEINZ: The mlx driver is part of stock Linux now.
        #               && (installed_ofed_driver("mlx4_ib", "$driver_subdir", "drivers/infiniband/hw/mlx4")
        #               && installed_ofed_driver("mlx4_core", "$driver_subdir", "drivers/net/mlx4"))
	    );
}

# only called if installed_intel_hfi is true
sub installed_version_intel_hfi()
{
    if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
	return `cat $ROOT$BASE_DIR/version_ofed`;
    } else {
	return "";
    }
}

# only called if available_intel_hfi is true
sub media_version_intel_hfi()
{
    return media_version_ofed();
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

    return (need_reinstall_ofed_comp('intel_hfi', $install_list, $installing_list));
}

sub preinstall_intel_hfi($$)
{
    my $install_list = shift();     # total that will be installed when done
    my $installing_list = shift();  # what items are being installed/reinstalled

    return preinstall_ofed("intel_hfi", $install_list, $installing_list);
}

sub install_intel_hfi($$)
{
    my $install_list = shift();     # total that will be installed when done
    my $installing_list = shift();  # what items are being installed/reinstalled

    print_install_banner_ofed_comp('intel_hfi');
    install_ofed_comp('intel_hfi', $install_list);

    need_reboot();
    $ComponentWasInstalled{'intel_hfi'}=1;
}

sub postinstall_intel_hfi($$)
{
    my $install_list = shift();     # total that will be installed when done
    my $installing_list = shift();  # what items are being installed/reinstalled
    ofed_restore_autostart('intel_hfi');
}

sub uninstall_intel_hfi($$)
{
    my $install_list = shift();     # total that will be left installed when done
    my $uninstalling_list = shift();        # what items are being uninstalled

    print_uninstall_banner_ofed_comp('intel_hfi');
        # TBD stop_intel_hfi;
    uninstall_ofed_comp('intel_hfi', $install_list, $uninstalling_list, 'verbose');
    need_reboot();
    $ComponentWasInstalled{'intel_hfi'}=0;
    remove_blacklist('hfi1');
}

# ==========================================================================
# ib_wfr_lite installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_ib_wfr_lite()
{
    return IsAutostart_ofed_comp2('ib_wfr_lite');
}
sub autostart_desc_ib_wfr_lite()
{
    return autostart_desc_ofed_comp('ib_wfr_lite');
}
# enable autostart for the given capability
sub enable_autostart2_ib_wfr_lite()
{
    enable_autostart_ofed_comp2('ib_wfr_lite');
}
# disable autostart for the given capability
sub disable_autostart2_ib_wfr_lite()
{
    disable_autostart_ofed_comp2('ib_wfr_lite');
}

sub available_ib_wfr_lite()
{
    my $srcdir=$ComponentInfo{'ib_wfr_lite'}{'SrcDir'};
        return ( (-d "$srcdir/SRPMS" || -d "$srcdir/RPMS" )
		 && build_option_is_allowed($CUR_OS_VER, "", %ofed_kernel_ib_options));
}

sub installed_ib_wfr_lite()
{
    my $driver_subdir=$ComponentInfo{'ib_wfr_lite'}{'DriverSubdir'};
        return (rpm_is_installed("ib_wfr_lite", "user")
                        && -e "$ROOT$BASE_DIR/version_ofed"
                        && rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER)
	    );
}

# only called if installed_ib_wfr_lite is true
sub installed_version_ib_wfr_lite()
{
    if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
	return `cat $ROOT$BASE_DIR/version_ofed`;
    } else {
	return "";
    }
}

# only called if available_ib_wfr_lite is true
sub media_version_ib_wfr_lite()
{
    return media_version_ofed();
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

    return (need_reinstall_ofed_comp('ib_wfr_lite', $install_list, $installing_list));
}

sub preinstall_ib_wfr_lite($$)
{
    my $install_list = shift();     # total that will be installed when done
    my $installing_list = shift();  # what items are being installed/reinstalled

    return preinstall_ofed("ib_wfr_lite", $install_list, $installing_list);
}

sub install_ib_wfr_lite($$)
{
    my $install_list = shift();     # total that will be installed when done
    my $installing_list = shift();  # what items are being installed/reinstalled

    print_install_banner_ofed_comp('ib_wfr_lite');
    install_ofed_comp('ib_wfr_lite', $install_list);

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
    ofed_restore_autostart('ib_wfr_lite');
}

sub uninstall_ib_wfr_lite($$)
{
    my $install_list = shift();     # total that will be left installed when done
    my $uninstalling_list = shift();        # what items are being uninstalled

    print_uninstall_banner_ofed_comp('ib_wfr_lite');
    uninstall_ofed_comp('ib_wfr_lite', $install_list, $uninstalling_list, 'verbose');

    # because the ib_wfr_lite and qib drivers are attached to the same hardware IDs
    # only one can be active in the system at a given time so blacklist the one that
    # won't currently be in use.
    add_blacklist("ib_wfr_lite");
    remove_blacklist("ib_qib");

    need_reboot();
    $ComponentWasInstalled{'ib_wfr_lite'}=0;
}

# ==========================================================================
# OFED ofed_mlx4 installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_ofed_mlx4()
{
	return IsAutostart_ofed_comp2('ofed_mlx4');
}
sub autostart_desc_ofed_mlx4()
{
	return autostart_desc_ofed_comp('ofed_mlx4');
}
# enable autostart for the given capability
sub enable_autostart2_ofed_mlx4()
{
	enable_autostart_ofed_comp2('ofed_mlx4');
}
# disable autostart for the given capability
sub disable_autostart2_ofed_mlx4()
{
	disable_autostart_ofed_comp2('ofed_mlx4');
}

# these are not presently called, which is good since ofed is hard to do this
#sub start_ofed_mlx4()
#{
#	my $driver_subdir=$ComponentInfo{'ofed_mlx4'}{'DriverSubdir'};
#	start_driver($ComponentInfo{'ofed_mlx4'}{'Name'}, "ib_core", $driver_subdir, "openibd");
#}

#sub stop_ofed_mlx4()
#{
#	stop_driver($ComponentInfo{'ofed_mlx4'}{'Name'}, "ib_core", "openibd");
#}

sub available_ofed_mlx4()
{
	my $srcdir=$ComponentInfo{'ofed_mlx4'}{'SrcDir'};
	return ( (-d "$srcdir/SRPMS" || -d "$srcdir/RPMS" )
			&& build_option_is_allowed($CUR_OS_VER, "--with-mlx4-mod", %ofed_kernel_ib_options));
}

sub installed_ofed_mlx4()
{
	my $driver_subdir=$ComponentInfo{'ofed_mlx4'}{'DriverSubdir'};
	return (rpm_is_installed("libmlx4", "user")
			&& -e "$ROOT$BASE_DIR/version_ofed"
			&& rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER)
	# MWHEINZ: The mlx driver is part of stock Linux now.
	#		&& (installed_ofed_driver("mlx4_ib", "$driver_subdir", "drivers/infiniband/hw/mlx4")
	#		&& installed_ofed_driver("mlx4_core", "$driver_subdir", "drivers/net/mlx4"))
			 );
}

# only called if installed_ofed_mlx4 is true
sub installed_version_ofed_mlx4()
{
	if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
		return `cat $ROOT$BASE_DIR/version_ofed`;
	} else {
		return "";
	}
}

# only called if available_ofed_mlx4 is true
sub media_version_ofed_mlx4()
{
	return media_version_ofed();
}

sub build_ofed_mlx4($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_ofed_mlx4($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_ofed_comp('ofed_mlx4', $install_list, $installing_list));
}

sub preinstall_ofed_mlx4($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("ofed_mlx4", $install_list, $installing_list);
}

sub install_ofed_mlx4($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('ofed_mlx4');
	install_ofed_comp('ofed_mlx4', $install_list);

	need_reboot();
	$ComponentWasInstalled{'ofed_mlx4'}=1;
}

sub postinstall_ofed_mlx4($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	ofed_restore_autostart('ofed_mlx4');
}

sub uninstall_ofed_mlx4($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('ofed_mlx4');
	# TBD stop_ofed_mlx4;
	uninstall_ofed_comp('ofed_mlx4', $install_list, $uninstalling_list, 'verbose');
	need_reboot();
	$ComponentWasInstalled{'ofed_mlx4'}=0;
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
	return (rpm_is_installed("libibverbs-devel", "user")
			&& -e "$ROOT$BASE_DIR/version_ofed");
}

# only called if installed_opa_stack_dev is true
sub installed_version_opa_stack_dev()
{
	if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
		return `cat $ROOT$BASE_DIR/version_ofed`;
	} else {
		return "";
	}
}

# only called if available_opa_stack_dev is true
sub media_version_opa_stack_dev()
{
	return media_version_ofed();
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

	return (need_reinstall_ofed_comp('opa_stack_dev', $install_list, $installing_list));
}

sub preinstall_opa_stack_dev($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("opa_stack_dev", $install_list, $installing_list);
}

sub install_opa_stack_dev($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('opa_stack_dev');
	install_ofed_comp('opa_stack_dev', $install_list);

	$ComponentWasInstalled{'opa_stack_dev'}=1;
}

sub postinstall_opa_stack_dev($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#ofed_restore_autostart('opa_stack_dev');
}

sub uninstall_opa_stack_dev($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('opa_stack_dev');
	uninstall_ofed_comp('opa_stack_dev', $install_list, $uninstalling_list, 'verbose');
	$ComponentWasInstalled{'opa_stack_dev'}=0;
}

# ==========================================================================
# OFED ofed_ipoib installation

my $FirstIPoIBInterface=0; # first device is ib0

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_ofed_ipoib()
{
	return IsAutostart_ofed_comp2('ofed_ipoib') || IsAutostart("ipoib");
}
sub autostart_desc_ofed_ipoib()
{
	return autostart_desc_ofed_comp('ofed_ipoib');
}
# enable autostart for the given capability
sub enable_autostart2_ofed_ipoib()
{
	enable_autostart_ofed_comp2('ofed_ipoib');
}
# disable autostart for the given capability
sub disable_autostart2_ofed_ipoib()
{
	disable_autostart_ofed_comp2('ofed_ipoib');
	if (Exist_ifcfg("ib")) {
		print "$ComponentInfo{'ofed_ipoib'}{'Name'} will autostart if ifcfg files exists\n";
		print "To fully disable autostart, it's recommended to also remove related ifcfg files\n";
		Remove_ifcfg("ib_ipoib","$ComponentInfo{'ofed_ipoib'}{'Name'}","ib");
	}
}

# these are not presently called, which is good since ofed is hard to do this
#sub start_ofed_ipoib()
#{
#	my $driver_subdir=$ComponentInfo{'ofed_ipoib'}{'DriverSubdir'};
#	start_driver($ComponentInfo{'ofed_ipoib'}{'Name'}, "ib_core", $driver_subdir, "openibd");
#}

#sub stop_ofed_ipoib()
#{
#	stop_driver($ComponentInfo{'ofed_ipoib'}{'Name'}, "ib_core", "openibd");
#}

sub available_ofed_ipoib()
{
	my $srcdir=$ComponentInfo{'ofed_ipoib'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_ofed_ipoib()
{
	my $driver_subdir=$ComponentInfo{'ofed_ipoib'}{'DriverSubdir'};
	return ((-e "$ROOT$BASE_DIR/version_ofed"
			&& rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER)));
}

# only called if installed_ofed_ipoib is true
sub installed_version_ofed_ipoib()
{
	return `cat $ROOT$BASE_DIR/version_ofed`;
}

# only called if available_ofed_ipoib is true
sub media_version_ofed_ipoib()
{
	return media_version_ofed();
}

sub build_ofed_ipoib($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_ofed_ipoib($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_ofed_comp('ofed_ipoib', $install_list, $installing_list));
}

sub preinstall_ofed_ipoib($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("ofed_ipoib", $install_list, $installing_list);
}

sub install_ofed_ipoib($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('ofed_ipoib');
	install_ofed_comp('ofed_ipoib', $install_list);

	prompt_openib_conf_param('SET_IPOIB_CM', 'IPoIB Connected Mode', "y", 'OPA_SET_IPOIB_CM');
	# bonding is more involved, require user to edit to enable that
	Config_ifcfg(1,"$ComponentInfo{'ofed_ipoib'}{'Name'}","ib", "$FirstIPoIBInterface",1);
	check_network_config;
	#Config_IPoIB_cfg;
	#add_insserv_conf("openibd", "network");
	need_reboot();
	$ComponentWasInstalled{'ofed_ipoib'}=1;
}

sub postinstall_ofed_ipoib($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	ofed_restore_autostart('ofed_ipoib');
}

sub uninstall_ofed_ipoib($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('ofed_ipoib');
	# TBD stop_ofed_ipoib;
	uninstall_ofed_comp('ofed_ipoib', $install_list, $uninstalling_list, 'verbose');
	#del_insserv_conf("openibd", "network");
	Remove_ifcfg("ib_ipoib","$ComponentInfo{'ofed_ipoib'}{'Name'}","ib");
	need_reboot();
	$ComponentWasInstalled{'ofed_ipoib'}=0;
}

# ==========================================================================
# OFED ofed_ib_bonding installation

# ib_bonding does not have any autostart configuration

sub available_ofed_ib_bonding()
{
	my $srcdir=$ComponentInfo{'ofed_ib_bonding'}{'SrcDir'};
	return ( (-d "$srcdir/SRPMS" || -d "$srcdir/RPMS")
			&& available_srpm("ib-bonding", $CUR_OS_VER, $CUR_OS_VER));
}

sub installed_ofed_ib_bonding()
{
	# TBD - should we detect OFED vs distro ib-bonding?
	return (-e "$ROOT$BASE_DIR/version_ofed"
			&& rpm_is_installed("ib-bonding", $CUR_OS_VER));
}

# only called if installed_ofed_ib_bonding is true
sub installed_version_ofed_ib_bonding()
{
	if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
		return `cat $ROOT$BASE_DIR/version_ofed`;
	} else {
		return "";
	}
}

# only called if available_ofed_ib_bonding is true
sub media_version_ofed_ib_bonding()
{
	return media_version_ofed();
}

sub build_ofed_ib_bonding($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_ofed_ib_bonding($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_ofed_comp('ofed_ib_bonding', $install_list, $installing_list));
}

sub preinstall_ofed_ib_bonding($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("ofed_ib_bonding", $install_list, $installing_list);
}

sub install_ofed_ib_bonding($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('ofed_ib_bonding');
	install_ofed_comp('ofed_ib_bonding', $install_list);

	need_reboot();
	$ComponentWasInstalled{'ofed_ib_bonding'}=1;
}

sub postinstall_ofed_ib_bonding($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#ofed_restore_autostart('ofed_ib_bonding');
}

sub uninstall_ofed_ib_bonding($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('ofed_ib_bonding');
	# TBD stop_ofed_ib_bonding;
	uninstall_ofed_comp('ofed_ib_bonding', $install_list, $uninstalling_list, 'verbose');
	need_reboot();
	$ComponentWasInstalled{'ofed_ib_bonding'}=0;
}

# ==========================================================================
# OFED ofed_udapl installation

sub available_ofed_udapl()
{
	my $srcdir=$ComponentInfo{'ofed_udapl'}{'SrcDir'};
	return ( (-d "$srcdir/SRPMS" || -d "$srcdir/RPMS" )
			&& available_srpm("dapl", "user", $CUR_OS_VER));
}

sub installed_ofed_udapl()
{
	return ((rpm_is_installed("dapl", "user")
			&& -e "$ROOT$BASE_DIR/version_ofed" ));
}

# only called if installed_ofed_udapl is true
sub installed_version_ofed_udapl()
{
	return `cat $ROOT$BASE_DIR/version_ofed`;
}

# only called if available_ofed_udapl is true
sub media_version_ofed_udapl()
{
	return media_version_ofed();
}

sub build_ofed_udapl($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_ofed_udapl($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_ofed_comp('ofed_udapl', $install_list, $installing_list));
}

sub preinstall_ofed_udapl($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("ofed_udapl", $install_list, $installing_list);
}

sub install_ofed_udapl($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('ofed_udapl');
	install_ofed_comp('ofed_udapl', $install_list);

	$ComponentWasInstalled{'ofed_udapl'}=1;
}

sub postinstall_ofed_udapl($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#ofed_restore_autostart('ofed_udapl');
}

sub uninstall_ofed_udapl($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('ofed_udapl');
	# TBD stop_ofed_udapl;
	uninstall_ofed_comp('ofed_udapl', $install_list, $uninstalling_list, 'verbose');
	$ComponentWasInstalled{'ofed_udapl'}=0;
}

# ==========================================================================
# OFED mpi-selector installation

sub available_mpi_selector()
{
	my $srcdir=$ComponentInfo{'mpi_selector'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_mpi_selector()
{
	return (rpm_is_installed("mpi-selector", "user")
			&& -e "$ROOT$BASE_DIR/version_ofed");
}

# only called if installed_mpi_selector is true
sub installed_version_mpi_selector()
{
	if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
		return `cat $ROOT$BASE_DIR/version_ofed`;
	} else {
		return "";
	}
}

# only called if available_mpi_selector is true
sub media_version_mpi_selector()
{
	return media_version_ofed();
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

	return (need_reinstall_ofed_comp('mpi_selector', $install_list, $installing_list));
}

sub check_os_prereqs_mpi_selector
{
 	return rpm_check_os_prereqs("mpi_selector", "any", ( "tcsh"))
}

sub preinstall_mpi_selector($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("mpi_selector", $install_list, $installing_list);
}

sub install_mpi_selector($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('mpi_selector');
	install_ofed_comp('mpi_selector', $install_list);

	$ComponentWasInstalled{'mpi_selector'}=1;
}

sub postinstall_mpi_selector($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#ofed_restore_autostart('mpi_selector');
}

sub uninstall_mpi_selector($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	uninstall_ofed_comp('mpiRest', $install_list, $uninstalling_list, 'verbose');
	print_uninstall_banner_ofed_comp('mpi_selector');
	uninstall_ofed_comp('mpi_selector', $install_list, $uninstalling_list, 'verbose');
	ofed_cleanup_mpitests();
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
			&& -e "$ROOT$BASE_DIR/version_ofed"));
}

# only called if installed_mvapich2 is true
sub installed_version_mvapich2()
{
	return `cat $ROOT$BASE_DIR/version_ofed`;
}

# only called if available_mvapich2 is true
sub media_version_mvapich2()
{
	return media_version_ofed();
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

	return (need_reinstall_ofed_comp('mvapich2', $install_list, $installing_list));
}

sub check_os_prereqs_mvapich2
{
 	return rpm_check_os_prereqs("mvapich2", "user", ( "sysfsutils", "libstdc++"));
}

sub preinstall_mvapich2($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("mvapich2", $install_list, $installing_list);
}

sub install_mvapich2($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('mvapich2');

	# make sure any old potentially custom built versions of mpi are uninstalled
	rpm_uninstall_list2("any", "--nodeps", 'silent', @{ $ofed_comp_info{'mvapich2'}{'UserRpms'}});
	my $rpmfile = rpm_resolve(ofed_rpms_dir(), "any", "mvapich2_gcc");
	if ( "$rpmfile" ne "" && -e "$rpmfile" ) {
		my $mpich_prefix= "$OFED_prefix/mpi/gcc/mvapich2-"
	   							. rpm_query_attr($rpmfile, "VERSION");
		if ( -d "$mpich_prefix" && GetYesNo ("Remove $mpich_prefix directory?", "y")) {
			LogPrint "rm -rf $mpich_prefix\n";
			system("rm -rf $mpich_prefix");
		}
	}

	install_ofed_comp('mvapich2', $install_list);

	$ComponentWasInstalled{'mvapich2'}=1;
}

sub postinstall_mvapich2($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#ofed_restore_autostart('mvapich2');
}

sub uninstall_mvapich2($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('mvapich2');
	uninstall_ofed_comp('mvapich2', $install_list, $uninstalling_list, 'verbose');
	ofed_cleanup_mpitests();
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
			&& -e "$ROOT$BASE_DIR/version_ofed"));
}

# only called if installed_openmpi is true
sub installed_version_openmpi()
{
	return `cat $ROOT$BASE_DIR/version_ofed`;
}

# only called if available_openmpi is true
sub media_version_openmpi()
{
	return media_version_ofed();
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

	return (need_reinstall_ofed_comp('openmpi', $install_list, $installing_list));
}

sub check_os_prereqs_openmpi
{
 	return rpm_check_os_prereqs("openmpi", "user", ( "libstdc++"));
}

sub preinstall_openmpi($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("openmpi", $install_list, $installing_list);
}

sub install_openmpi($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('openmpi');

	# make sure any old potentially custom built versions of mpi are uninstalled
	rpm_uninstall_list2("any", "--nodeps", 'silent', @{ $ofed_comp_info{'openmpi'}{'UserRpms'}});
	my $rpmfile = rpm_resolve(ofed_rpms_dir(), "any", "openmpi_gcc");
	if ( "$rpmfile" ne "" && -e "$rpmfile" ) {
		my $mpich_prefix= "$OFED_prefix/mpi/gcc/openmpi-"
	   							. rpm_query_attr($rpmfile, "VERSION");
		if ( -d "$mpich_prefix" && GetYesNo ("Remove $mpich_prefix directory?", "y")) {
			LogPrint "rm -rf $mpich_prefix\n";
			system("rm -rf $mpich_prefix");
		}
	}

	install_ofed_comp('openmpi', $install_list);

	$ComponentWasInstalled{'openmpi'}=1;
}

sub postinstall_openmpi($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#ofed_restore_autostart('openmpi');
}

sub uninstall_openmpi($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('openmpi');
	uninstall_ofed_comp('openmpi', $install_list, $uninstalling_list, 'verbose');
	ofed_cleanup_mpitests();
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
			&& -e "$ROOT$BASE_DIR/version_ofed"));
}

# only called if installed_gasnet is true
sub installed_version_gasnet()
{
	return `cat $ROOT$BASE_DIR/version_ofed`;
}

# only called if available_gasnet is true
sub media_version_gasnet()
{
	return media_version_ofed();
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

	return (need_reinstall_ofed_comp('gasnet', $install_list, $installing_list));
}

sub preinstall_gasnet($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("gasnet", $install_list, $installing_list);
}

sub install_gasnet($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('gasnet');

	# make sure any old potentially custom built versions of mpi are uninstalled
	rpm_uninstall_list2("any", "--nodeps", 'silent', @{ $ofed_comp_info{'gasnet'}{'UserRpms'}});
	my $rpmfile = rpm_resolve(ofed_rpms_dir(), "any", "gasnet");
	if ( "$rpmfile" ne "" && -e "$rpmfile" ) {
		my $mpich_prefix= "$OFED_prefix/shmem/gcc/gasnet-"
	   							. rpm_query_attr($rpmfile, "VERSION");
		if ( -d "$mpich_prefix" && GetYesNo ("Remove $mpich_prefix directory?", "y")) {
			LogPrint "rm -rf $mpich_prefix\n";
			system("rm -rf $mpich_prefix");
		}
	}

	install_ofed_comp('gasnet', $install_list);

	$ComponentWasInstalled{'gasnet'}=1;
}

sub postinstall_gasnet($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#ofed_restore_autostart('gasnet');
}

sub uninstall_gasnet($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('gasnet');
	uninstall_ofed_comp('gasnet', $install_list, $uninstalling_list, 'verbose');
#	ofed_cleanup_mpitests();
	$ComponentWasInstalled{'gasnet'}=0;
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
			&& -e "$ROOT$BASE_DIR/version_ofed"));
}

# only called if installed_openshmem is true
sub installed_version_openshmem()
{
	return `cat $ROOT$BASE_DIR/version_ofed`;
}

# only called if available_openshmem is true
sub media_version_openshmem()
{
	return media_version_ofed();
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

	return (need_reinstall_ofed_comp('openshmem', $install_list, $installing_list));
}

sub preinstall_openshmem($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("openshmem", $install_list, $installing_list);
}

sub install_openshmem($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('openshmem');

	# make sure any old potentially custom built versions of mpi are uninstalled
	rpm_uninstall_list2("any", "--nodeps", 'silent', @{ $ofed_comp_info{'openshmem'}{'UserRpms'}});
	my $rpmfile = rpm_resolve(ofed_rpms_dir(), "any", "openshmem");
	if ( "$rpmfile" ne "" && -e "$rpmfile" ) {
		my $mpich_prefix= "$OFED_prefix/shmem/gcc/openshmem-"
	   							. rpm_query_attr($rpmfile, "VERSION");
		if ( -d "$mpich_prefix" && GetYesNo ("Remove $mpich_prefix directory?", "y")) {
			LogPrint "rm -rf $mpich_prefix\n";
			system("rm -rf $mpich_prefix");
		}
	}

	install_ofed_comp('openshmem', $install_list);

	$ComponentWasInstalled{'openshmem'}=1;
}

sub postinstall_openshmem($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#ofed_restore_autostart('openshmem');
}

sub uninstall_openshmem($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('openshmem');
	uninstall_ofed_comp('openshmem', $install_list, $uninstalling_list, 'verbose');
#	ofed_cleanup_mpitests();
	$ComponentWasInstalled{'openshmem'}=0;
}

# ==========================================================================
# OFED ofed_mpisrc installation

sub available_ofed_mpisrc()
{
	my $srcdir=$ComponentInfo{'ofed_mpisrc'}{'SrcDir'};
# TBD better checks for available?
# check file_glob("$srcdir/SRPMS/mvapich-*.src.rpm") ne ""
# check file_glob("$srcdir/SRPMS/mvapich2-*.src.rpm") ne ""
# check file_glob("$srcdir/SRPMS/openmpi-*.src.rpm") ne ""
	return ( (-d "$srcdir/SRPMS" || -d "$srcdir/RPMS" ) );
}

sub installed_ofed_mpisrc()
{
	return ((-e "$ROOT$BASE_DIR/version_ofed"
			&& file_glob("$ROOT/usr/src/opa/MPI/mvapich*.src.rpm") ne ""
			&& file_glob("$ROOT/usr/src/opa/MPI/openmpi*.src.rpm") ne ""
			&& file_glob("$ROOT/usr/src/opa/MPI/mpitests*.src.rpm") ne ""));
}

# only called if installed_ofed_mpisrc is true
sub installed_version_ofed_mpisrc()
{
	return `cat $ROOT$BASE_DIR/version_ofed`;
}

# only called if available_ofed_mpisrc is true
sub media_version_ofed_mpisrc()
{
	return media_version_ofed();
}

sub build_ofed_mpisrc($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_ofed_mpisrc($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_ofed_comp('ofed_mpisrc', $install_list, $installing_list));
}

sub check_os_prereqs_ofed_mpisrc
{
 	return rpm_check_os_prereqs("ofed_mpisrc", "any", 
					( @{ $ofed_srpm_info{'mvapich'}{'BuildPrereq'}},
					  @{ $ofed_srpm_info{'openmpi'}{'BuildPrereq'}},
					  @{ $ofed_srpm_info{'mvapich2'}{'BuildPrereq'}} ));
}

sub preinstall_ofed_mpisrc($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("ofed_mpisrc", $install_list, $installing_list);
}

sub install_ofed_mpisrc($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	my $srcdir=$ComponentInfo{'ofed_mpisrc'}{'SrcDir'};

	print_install_banner_ofed_comp('ofed_mpisrc');
	install_ofed_comp('ofed_mpisrc', $install_list);
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

	$ComponentWasInstalled{'ofed_mpisrc'}=1;
}

sub postinstall_ofed_mpisrc($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#ofed_restore_autostart('ofed_mpisrc');
}

sub uninstall_ofed_mpisrc($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('ofed_mpisrc');

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

	uninstall_ofed_comp('ofed_mpisrc', $install_list, $uninstalling_list, 'verbose');
	system "rmdir $ROOT/usr/src/opa/MPI 2>/dev/null"; # remove only if empty
	system "rmdir $ROOT/usr/src/opa 2>/dev/null"; # remove only if empty
	$ComponentWasInstalled{'ofed_mpisrc'}=0;
}

# ==========================================================================
# OFED ofed_srp installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_ofed_srp()
{
	return IsAutostart_ofed_comp2('ofed_srp');
}
sub autostart_desc_ofed_srp()
{
	return autostart_desc_ofed_comp('ofed_srp');
}
# enable autostart for the given capability
sub enable_autostart2_ofed_srp()
{
	enable_autostart_ofed_comp2('ofed_srp');
}
# disable autostart for the given capability
sub disable_autostart2_ofed_srp()
{
	disable_autostart_ofed_comp2('ofed_srp');
}

# these are not presently called, which is good since ofed is hard to do this
#sub start_ofed_srp()
#{
#	my $driver_subdir=$ComponentInfo{'ofed_srp'}{'DriverSubdir'};
#	start_driver($ComponentInfo{'ofed_srp'}{'Name'}, "ib_core", $driver_subdir, "openibd");
#}

#sub stop_ofed_srp()
#{
#	stop_driver($ComponentInfo{'ofed_srp'}{'Name'}, "ib_core", "openibd");
#}

sub available_ofed_srp()
{
	my $srcdir=$ComponentInfo{'ofed_srp'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_ofed_srp()
{
	my $driver_subdir=$ComponentInfo{'ofed_srp'}{'DriverSubdir'};
	return (rpm_is_installed("srptools", "user")
			&& -e "$ROOT$BASE_DIR/version_ofed"
			&& rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER)
	# MWHEINZ: using stock srp.
	#		&& installed_ofed_driver("ib_srp", "$driver_subdir", "drivers/infiniband/ulp/srp")
			 );
}

# only called if installed_ofed_srp is true
sub installed_version_ofed_srp()
{
	if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
		return `cat $ROOT$BASE_DIR/version_ofed`;
	} else {
		return "";
	}
}

# only called if available_ofed_srp is true
sub media_version_ofed_srp()
{
	return media_version_ofed();
}

sub build_ofed_srp($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_ofed_srp($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_ofed_comp('ofed_srp', $install_list, $installing_list));
}

sub preinstall_ofed_srp($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("ofed_srp", $install_list, $installing_list);
}

sub install_ofed_srp($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('ofed_srp');
	install_ofed_comp('ofed_srp', $install_list);

	prompt_openib_conf_param('SRPHA_ENABLE', 'OFA SRP High Availability deamon', "n", 'OPA_SRPHA_ENABLE');
	need_reboot();
	$ComponentWasInstalled{'ofed_srp'}=1;
}

sub postinstall_ofed_srp($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	ofed_restore_autostart('ofed_srp');
}

sub uninstall_ofed_srp($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('ofed_srp');
	# TBD stop_ofed_srp;
	uninstall_ofed_comp('ofed_srp', $install_list, $uninstalling_list, 'verbose');
	need_reboot();
	$ComponentWasInstalled{'ofed_srp'}=0;
}

# ==========================================================================
# OFED ofed_srpt installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_ofed_srpt()
{
	return IsAutostart_ofed_comp2('ofed_srpt');
}
sub autostart_desc_ofed_srpt()
{
	return autostart_desc_ofed_comp('ofed_srpt');
}
# enable autostart for the given capability
sub enable_autostart2_ofed_srpt()
{
	enable_autostart_ofed_comp2('ofed_srpt');
}
# disable autostart for the given capability
sub disable_autostart2_ofed_srpt()
{
	disable_autostart_ofed_comp2('ofed_srpt');
}

# these are not presently called, which is good since ofed is hard to do this
#sub start_ofed_srpt()
#{
#	my $driver_subdir=$ComponentInfo{'ofed_srpt'}{'DriverSubdir'};
#	start_driver($ComponentInfo{'ofed_srpt'}{'Name'}, "ib_core", $driver_subdir, "openibd");
#}

#sub stop_ofed_srpt()
#{
#	stop_driver($ComponentInfo{'ofed_srpt'}{'Name'}, "ib_core", "openibd");
#}

sub available_ofed_srpt()
{
	my $srcdir=$ComponentInfo{'ofed_srpt'}{'SrcDir'};
	return ( (-d "$srcdir/SRPMS" || -d "$srcdir/RPMS" )
			 && build_option_is_allowed($CUR_OS_VER, "--with-srp-target-mod", %ofed_kernel_ib_options));
}

sub installed_ofed_srpt()
{
	my $driver_subdir=$ComponentInfo{'ofed_srpt'}{'DriverSubdir'};
	return (-e "$ROOT$BASE_DIR/version_ofed"
			&& rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER)
	# MWHEINZ: Using stock srpt.
	#		&& installed_ofed_driver("ib_srpt", "$driver_subdir", "drivers/infiniband/ulp/srpt")
			 );
}

# only called if installed_ofed_srpt is true
sub installed_version_ofed_srpt()
{
	if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
		return `cat $ROOT$BASE_DIR/version_ofed`;
	} else {
		return "";
	}
}

# only called if available_ofed_srpt is true
sub media_version_ofed_srpt()
{
	return media_version_ofed();
}

sub build_ofed_srpt($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_ofed_srpt($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_ofed_comp('ofed_srpt', $install_list, $installing_list));
}

sub preinstall_ofed_srpt($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("ofed_srpt", $install_list, $installing_list);
}

sub install_ofed_srpt($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('ofed_srpt');
	install_ofed_comp('ofed_srpt', $install_list);

	need_reboot();
	$ComponentWasInstalled{'ofed_srpt'}=1;
}

sub postinstall_ofed_srpt($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	ofed_restore_autostart('ofed_srpt');
}

sub uninstall_ofed_srpt($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('ofed_srpt');
	# TBD stop_ofed_srpt;
	uninstall_ofed_comp('ofed_srpt', $install_list, $uninstalling_list, 'verbose');
	need_reboot();
	$ComponentWasInstalled{'ofed_srpt'}=0;
}

# ==========================================================================
# OFED ofed_iser installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_ofed_iser()
{
	return IsAutostart_ofed_comp2('ofed_iser');
}
sub autostart_desc_ofed_iser()
{
	return autostart_desc_ofed_comp('ofed_iser');
}
# enable autostart for the given capability
sub enable_autostart2_ofed_iser()
{
	enable_autostart_ofed_comp2('ofed_iser');
}
# disable autostart for the given capability
sub disable_autostart2_ofed_iser()
{
	disable_autostart_ofed_comp2('ofed_iser');
}

# these are not presently called, which is good since ofed is hard to do this
#sub start_ofed_iser()
#{
#	my $driver_subdir=$ComponentInfo{'ofed_iser'}{'DriverSubdir'};
#	start_driver($ComponentInfo{'ofed_iser'}{'Name'}, "ib_core", $driver_subdir, "openibd");
#}

#sub stop_ofed_iser()
#{
#	stop_driver($ComponentInfo{'ofed_iser'}{'Name'}, "ib_core", "openibd");
#}

sub available_ofed_iser()
{
	my $srcdir=$ComponentInfo{'ofed_iser'}{'SrcDir'};
	return ( (-d "$srcdir/SRPMS" || -d "$srcdir/RPMS" )
			&& build_option_is_allowed($CUR_OS_VER, "--with-iser-mod", %ofed_kernel_ib_options));
}

sub installed_ofed_iser()
{
	my $driver_subdir=$ComponentInfo{'ofed_iser'}{'DriverSubdir'};
	return (
			-e "$ROOT$BASE_DIR/version_ofed"
			&& rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER)
			# MWHEINZ using stock iser. 
			# && installed_ofed_driver("ib_iser", "$driver_subdir", "drivers/infiniband/ulp/iser")
			 );
}

# only called if installed_ofed_iser is true
sub installed_version_ofed_iser()
{
	if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
		return `cat $ROOT$BASE_DIR/version_ofed`;
	} else {
		return "";
	}
}

# only called if available_ofed_iser is true
sub media_version_ofed_iser()
{
	return media_version_ofed();
}

sub build_ofed_iser($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_ofed_iser($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_ofed_comp('ofed_iser', $install_list, $installing_list));
}

sub preinstall_ofed_iser($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("ofed_iser", $install_list, $installing_list);
}

sub install_ofed_iser($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('ofed_iser');
	install_ofed_comp('ofed_iser', $install_list);

	need_reboot();
	$ComponentWasInstalled{'ofed_iser'}=1;
}

sub postinstall_ofed_iser($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	ofed_restore_autostart('ofed_iser');
}

sub uninstall_ofed_iser($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('ofed_iser');
	# TBD stop_ofed_iser;
	uninstall_ofed_comp('ofed_iser', $install_list, $uninstalling_list, 'verbose');
	need_reboot();
	$ComponentWasInstalled{'ofed_iser'}=0;
}

# ==========================================================================
# OFED ofed_nfsrdma installation

# involves /etc/init.d/nfs and /etc/init.d/nfslock
# The HasStart is disabled, so these won't be called.
# TBD - If this is enabled, what return value to use and how should it
# interact with the two daemons
# should we treat nfs as a subcomponent much like we used to handle opamon?
# autostart functions are per subcomponent
sub IsAutostart2_ofed_nfsrdma()
{
	return IsAutostart("nfslock") || IsAutoStart("nfs");
}
sub autostart_desc_ofed_nfsrdma()
{
	return "$ComponentInfo{'ofed_nfsrdma'}{'Name'}";
}
sub enable_autostart2_ofed_nfsrdma()
{
	enable_autostart("nfslock");
	enable_autostart("nfs");
}
sub disable_autostart2_ofed_nfsrdma()
{
	disable_autostart("nfs");
	disable_autostart("nfslock");
}

sub start_ofed_nfsrdma()
{
	my $driver_subdir=$ComponentInfo{'ofed_nfsrdma'}{'DriverSubdir'};
	start_driver($ComponentInfo{'ofed_nfsrdma'}{'Name'}, "lockd", "$driver_subdir/fs/lockd", "nfslock");
	start_driver($ComponentInfo{'ofed_nfsrdma'}{'Name'}, "nfs", "$driver_subdir/fs/nfs", "nfs");
}

sub stop_ofed_nfsrdma()
{
	stop_driver($ComponentInfo{'ofed_nfsrdma'}{'Name'}, "nfs", "nfs");
	stop_driver($ComponentInfo{'ofed_nfsrdma'}{'Name'}, "lockd", "nfslock");
}

sub available_ofed_nfsrdma()
{
	my $srcdir=$ComponentInfo{'ofed_nfsrdma'}{'SrcDir'};
	return ( (-d "$srcdir/SRPMS" || -d "$srcdir/RPMS")
			&& build_option_is_allowed($CUR_OS_VER, "--with-nfsrdma-mod", %ofed_kernel_ib_options));
}

sub installed_ofed_nfsrdma()
{
	my $driver_subdir=$ComponentInfo{'ofed_nfsrdma'}{'DriverSubdir'};
	return (#rpm_is_installed("rnfs-utils", "user") &&
			-e "$ROOT$BASE_DIR/version_ofed"
			&& rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER)
			# MWHEINZ using stock driver. 
			# && installed_ofed_driver("xprtrdma", "$driver_subdir", "net/sunrpc/xprtrdma")
			 );
}

# only called if installed_ofed_nfsrdma is true
sub installed_version_ofed_nfsrdma()
{
	if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
		return `cat $ROOT$BASE_DIR/version_ofed`;
	} else {
		return "";
	}
}

# only called if available_ofed_nfsrdma is true
sub media_version_ofed_nfsrdma()
{
	return media_version_ofed();
}

sub build_ofed_nfsrdma($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_ofed_nfsrdma($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_ofed_comp('ofed_nfsrdma', $install_list, $installing_list));
}

sub preinstall_ofed_nfsrdma($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("ofed_nfsrdma", $install_list, $installing_list);
}

sub install_ofed_nfsrdma($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('ofed_nfsrdma');
	install_ofed_comp('ofed_nfsrdma', $install_list);

	need_reboot();
	$ComponentWasInstalled{'ofed_nfsrdma'}=1;
}

sub postinstall_ofed_nfsrdma($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	# no need to restore_autostart, ofed reinstall does not affect nfs nor nfslock autostart
}

sub uninstall_ofed_nfsrdma($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('ofed_nfsrdma');
	# TBD stop_ofed_nfsrdma;
	uninstall_ofed_comp('ofed_nfsrdma', $install_list, $uninstalling_list, 'verbose');
	need_reboot();
	$ComponentWasInstalled{'ofed_nfsrdma'}=0;
}

# ==========================================================================
# OFED ofed_iwarp installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_ofed_iwarp()
{
	return IsAutostart_ofed_comp2('ofed_iwarp');
}
sub autostart_desc_ofed_iwarp()
{
	return autostart_desc_ofed_comp('ofed_iwarp');
}
# enable autostart for the given capability
sub enable_autostart2_ofed_iwarp()
{
	enable_autostart_ofed_comp2('ofed_iwarp');
}
# disable autostart for the given capability
sub disable_autostart2_ofed_iwarp()
{
	disable_autostart_ofed_comp2('ofed_iwarp');
}

# these are not presently called, which is good since ofed is hard to do this
#sub start_ofed_iwarp()
#{
#	my $driver_subdir=$ComponentInfo{'ofed_iwarp'}{'DriverSubdir'};
#	start_driver($ComponentInfo{'ofed_iwarp'}{'Name'}, "ib_core", $driver_subdir, "openibd");
#}

#sub stop_ofed_iwarp()
#{
#	stop_driver($ComponentInfo{'ofed_iwarp'}{'Name'}, "ib_core", "openibd");
#}

sub available_ofed_iwarp()
{
	my $srcdir=$ComponentInfo{'ofed_iwarp'}{'SrcDir'};
	return ( (-d "$srcdir/SRPMS" || -d "$srcdir/RPMS" )
			&& (build_option_is_allowed($CUR_OS_VER, "--with-cxgb3-mod", %ofed_kernel_ib_options)
			|| build_option_is_allowed($CUR_OS_VER, "--with-cxgb4-mod", %ofed_kernel_ib_options)
			|| build_option_is_allowed($CUR_OS_VER, "--with-nes-mod", %ofed_kernel_ib_options)));
}

sub installed_ofed_iwarp()
{
	my $driver_subdir=$ComponentInfo{'ofed_iwarp'}{'DriverSubdir'};
	return ((rpm_is_installed("libcxgb3", "user")
				|| rpm_is_installed("libcxgb4", "user")
				|| rpm_is_installed("libnes", "user"))
			&& -e "$ROOT$BASE_DIR/version_ofed"
			&& rpm_is_installed("ifs-kernel-updates", $CUR_OS_VER)
			# MWHEINZ using stock drivers.
			# && (installed_ofed_driver("iw_cxgb3", "$driver_subdir", "drivers/infiniband/hw/cxgb3")
			#	|| installed_ofed_driver("iw_cxgb4", "$driver_subdir", "drivers/infiniband/hw/cxgb4")
			#	|| installed_ofed_driver("iw_nes", "$driver_subdir", "drivers/infiniband/hw/nes"))
			 );
}

# only called if installed_ofed_iwarp is true
sub installed_version_ofed_iwarp()
{
	if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
		return `cat $ROOT$BASE_DIR/version_ofed`;
	} else {
		return "";
	}
}

# only called if available_ofed_iwarp is true
sub media_version_ofed_iwarp()
{
	return media_version_ofed();
}

sub build_ofed_iwarp($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_ofed_iwarp($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_ofed_comp('ofed_iwarp', $install_list, $installing_list));
}

sub preinstall_ofed_iwarp($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("ofed_iwarp", $install_list, $installing_list);
}

sub install_ofed_iwarp($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('ofed_iwarp');
	install_ofed_comp('ofed_iwarp', $install_list);

	need_reboot();
	$ComponentWasInstalled{'ofed_iwarp'}=1;
}

sub postinstall_ofed_iwarp($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	ofed_restore_autostart('ofed_iwarp');
}

sub uninstall_ofed_iwarp($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('ofed_iwarp');
	# TBD stop_ofed_iwarp;
	uninstall_ofed_comp('ofed_iwarp', $install_list, $uninstalling_list, 'verbose');
	need_reboot();
	$ComponentWasInstalled{'ofed_iwarp'}=0;
}

# ==========================================================================
# OFED opensm installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_opensm()
{
	return IsAutostart_ofed_comp2('opensm');
}
sub autostart_desc_opensm()
{
	return autostart_desc_ofed_comp('opensm');
}
# enable autostart for the given capability
sub enable_autostart2_opensm()
{
	enable_autostart($ofed_comp_info{'opensm'}{'StartupScript'});
}
# disable autostart for the given capability
sub disable_autostart2_opensm()
{
	disable_autostart($ofed_comp_info{'opensm'}{'StartupScript'});
}

sub start_opensm()
{
	my $prefix = ofed_get_prefix();
	start_utility($ComponentInfo{'opensm'}{'Name'}, "$prefix/sbin", "opensm", "opensmd");
}

sub stop_opensm()
{
	stop_utility($ComponentInfo{'opensm'}{'Name'}, "opensm", "opensmd");
}

sub available_opensm()
{
	my $srcdir=$ComponentInfo{'opensm'}{'SrcDir'};
	return ( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS" );
}

sub installed_opensm()
{
	return (rpm_is_installed("opensm", "user")
			&& -e "$ROOT$BASE_DIR/version_ofed");
}

# only called if installed_opensm is true
sub installed_version_opensm()
{
	if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
		return `cat $ROOT$BASE_DIR/version_ofed`;
	} else {
		return "";
	}
}

# only called if available_opensm is true
sub media_version_opensm()
{
	return media_version_ofed();
}

sub build_opensm($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_opensm($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return (need_reinstall_ofed_comp('opensm', $install_list, $installing_list));
}

sub preinstall_opensm($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("opensm", $install_list, $installing_list);
}

sub install_opensm($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('opensm');
	install_ofed_comp('opensm', $install_list);

	$ComponentWasInstalled{'opensm'}=1;
}

sub postinstall_opensm($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	ofed_restore_autostart('opensm');
}

sub uninstall_opensm($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('opensm');
	stop_opensm;

	uninstall_ofed_comp('opensm', $install_list, $uninstalling_list, 'verbose');
	$ComponentWasInstalled{'opensm'}=0;
}

# ==========================================================================
# OFED ofed_debug installation

# this is an odd component.  It consists of the debuginfo files which
# are built and identified in DebugRpms in other components.  Installing this
# component installs the debuginfo files for the installed components.
# uninstalling this component gets rid of all debuginfo files.
# uninstalling other components will get rid of individual debuginfo files
# for those components

sub available_ofed_debug()
{
	my $srcdir=$ComponentInfo{'ofed_debug'}{'SrcDir'};
	return (( -d "$srcdir/SRPMS" || -d "$srcdir/RPMS")
			&&	$ofed_rpm_info{'libibverbs-debuginfo'}{'Available'});
}

sub installed_ofed_debug()
{
	return (rpm_is_installed("libibverbs-debuginfo", "user")
			&& -e "$ROOT$BASE_DIR/version_ofed");
}

# only called if installed_ofed_debug is true
sub installed_version_ofed_debug()
{
	if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
		return `cat $ROOT$BASE_DIR/version_ofed`;
	} else {
		return "";
	}
}

# only called if available_ofed_debug is true
sub media_version_ofed_debug()
{
	return media_version_ofed();
}

sub build_ofed_debug($$$$)
{
	my $osver = shift();
	my $debug = shift();	# enable extra debug of build itself
	my $build_temp = shift();	# temp area for use by build
	my $force = shift();	# force a rebuild
	return 0;	# success
}

sub need_reinstall_ofed_debug($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	my $reins = need_reinstall_ofed_comp('ofed_debug', $install_list, $installing_list);
	if ("$reins" eq "no" ) {
		# if ofed components with DebugRpms have been added we need to reinstall
		# this component.  Note uninstall for individual components will
		# get rid of associated debuginfo files
		foreach my $comp ( @ofed_components ) {
			if ( " $installing_list " =~ / $comp /
				 && 0 != scalar(@{ $ofed_comp_info{$comp}{'DebugRpms'}})) {
				return "this";
			}
		}
		
	}
	return $reins;
}

sub preinstall_ofed_debug($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("ofed_debug", $install_list, $installing_list);
}

sub install_ofed_debug($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	my @list;

	print_install_banner_ofed_comp('ofed_debug');
	install_ofed_comp('ofed_debug', $install_list);

	# install DebugRpms for each installed component
	foreach my $comp ( @ofed_components ) {
		if ( " $install_list " =~ / $comp / ) {
			ofed_rpm_install_list(ofed_rpms_dir(), $CUR_OS_VER, 1,
							( @{ $ofed_comp_info{$comp}{'DebugRpms'}}));
		}
	}

	$ComponentWasInstalled{'ofed_debug'}=1;
}

sub postinstall_ofed_debug($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	#ofed_restore_autostart('ofed_debug');
}

sub uninstall_ofed_debug($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('ofed_debug');

	uninstall_ofed_comp('ofed_debug', $install_list, $uninstalling_list, 'verbose');
	# uninstall debug rpms for all components
	# debuginfo never in >1 component, so do explicit uninstall since
	# have an odd PartOf relationship which confuses uninstall_not_needed_list
	foreach my $comp ( reverse(@ofed_components) ) {
		if ( "$comp" eq "ofed_ib_bonding" && ! $ofed_rpm_info{'ib-bonding'}{'Available'}) {
			# skip, might be included with distro
		} else {
			rpm_uninstall_list("any", 'verbose',
					 @{ $ofed_comp_info{$comp}{'DebugRpms'}});
		}
	}
	$ComponentWasInstalled{'ofed_debug'}=0;
}

# ==========================================================================
# OFED ibacm installation

# determine if the given capability is configured for Autostart at boot
sub IsAutostart2_ibacm()
{
	return IsAutostart_ofed_comp2('ibacm');
}
sub autostart_desc_ibacm()
{
	return autostart_desc_ofed_comp('ibacm');
}
# enable autostart for the given capability
sub enable_autostart2_ibacm()
{
	enable_autostart($ofed_comp_info{'ibacm'}{'StartupScript'});
}
# disable autostart for the given capability
sub disable_autostart2_ibacm()
{
	disable_autostart($ofed_comp_info{'ibacm'}{'StartupScript'});
}

sub start_ibacm()
{
	my $prefix = ofed_get_prefix();
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
			&& -e "$ROOT$BASE_DIR/version_ofed");
}

# only called if installed_ibacm is true
sub installed_version_ibacm()
{
	if ( -e "$ROOT$BASE_DIR/version_ofed" ) {
		return `cat $ROOT$BASE_DIR/version_ofed`;
	} else {
		return "";
	}
}

# only called if available_ibacm is true
sub media_version_ibacm()
{
	return media_version_ofed();
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

	return (need_reinstall_ofed_comp('ibacm', $install_list, $installing_list));
}

sub preinstall_ibacm($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return preinstall_ofed("ibacm", $install_list, $installing_list);
}

sub install_ibacm($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	print_install_banner_ofed_comp('ibacm');
	install_ofed_comp('ibacm', $install_list);

	$ComponentWasInstalled{'ibacm'}=1;
}

sub postinstall_ibacm($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled
	ofed_restore_autostart('ibacm');
}

sub uninstall_ibacm($$)
{
	my $install_list = shift();	# total that will be left installed when done
	my $uninstalling_list = shift();	# what items are being uninstalled

	print_uninstall_banner_ofed_comp('ibacm');
	stop_ibacm;

	uninstall_ofed_comp('ibacm', $install_list, $uninstalling_list, 'verbose');
	$ComponentWasInstalled{'ibacm'}=0;
}
