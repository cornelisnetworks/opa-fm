#!/usr/bin/perl
## BEGIN_ICS_COPYRIGHT8 ****************************************
# 
# Copyright (c) 2015-2018, Intel Corporation
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
## END_ICS_COPYRIGHT8   ****************************************
#
## [ICS VERSION STRING: unknown]
#use strict;
##use Term::ANSIColor;
##use Term::ANSIColor qw(:constants);
##use File::Basename;
##use Math::BigInt;
#
## ==========================================================================
#
#Installation Prequisites array for delta components
my @opa_stack_prereq = (
			"bash",
			"kmod",
			"rdma-core",
			"rdma-ndd",
			"systemd",
			"coreutils",
			"grep",
			"opensm-libs3",
			"libibmad5",
			"libibcm1",
			"libibumad3",
		        "rdma-core-devel",
);
$comp_prereq_hash{'opa_stack_prereq'} = \@opa_stack_prereq;

my @intel_hfi_prereq = (
			"glibc",
			"libgcc_s1",
			"bash",
			"udev",
			"libudev-devel",
			"python-base",
			"libedit0",
			"libncurses5",
			"libnuma1",
			"irqbalance",
);
$comp_prereq_hash{'intel_hfi_prereq'} = \@intel_hfi_prereq;

my @mvapich2_gcc_hfi_prereq = (
			"bash",
			"glibc",
			"libz1",
			"mpi-selector",
);
$comp_prereq_hash{'mvapich2_gcc_hfi_prereq'} = \@mvapich2_gcc_hfi_prereq;

my @mvapich2_intel_hfi_prereq = (
			"bash",
			"mpi-selector",
);
$comp_prereq_hash{'mvapich2_intel_hfi_prereq'} = \@mvapich2_intel_hfi_prereq;

my @openmpi_gcc_hfi_prereq = (
			"glibc",
			"bash",
			"libpsm_infinipath1",
			"pkg-config",
			"libgcc_s1",
			"libgfortran3",
			"gcc-fortran",
			"libgomp1",
			"libibverbs1",
			"libquadmath0",
			"librdmacm1",
			"libstdc++6",
			"libz1",
			"opensm-libs3",
			"opensm-devel",
			"mpi-selector",
);
$comp_prereq_hash{'openmpi_gcc_hfi_prereq'} = \@openmpi_gcc_hfi_prereq;

my @openmpi_intel_hfi_prereq = (
			"bash",
			"mpi-selector",
);
$comp_prereq_hash{'openmpi_intel_hfi_prereq'} = \@openmpi_intel_hfi_prereq;

my @mvapich2_prereq = (
			"bash",
			"libibverbs1",
			"librdmacm1",
			"glibc",
			"libz1",
			"mpi-selector",
);
$comp_prereq_hash{'mvapich2_prereq'} = \@mvapich2_prereq;

my @openmpi_prereq = (
			"glibc",
			"bash",
			"libz1",
			"pkg-config",
			"libgcc_s1",
			"libgfortran3",
			"gcc-fortran",
			"libgomp1",
			"libibverbs1",
			"libquadmath0",
			"librdmacm1",
			"libstdc++6",
			"libz1",
			"opensm-libs3",
			"opensm-devel",
			"mpi-selector",
);
$comp_prereq_hash{'openmpi_prereq'} = \@openmpi_prereq;
