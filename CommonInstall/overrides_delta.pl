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

	# Names of supported install components
	# must be listed in depdency order such that prereqs appear 1st
# TBD ofed_vnic
my @delta_Components_other = ( "opa_stack", "ibacm", "intel_hfi", "mpi_selector",
		"delta_ipoib",
		"opa_stack_dev", "rdma_ndd",
 		"gasnet", "openshmem",
		"mvapich2", "openmpi",
		"delta_mpisrc",
		"hfi1_uefi",
		"delta_debug", );
my @delta_Components_rhel72 = ( "opa_stack", "ibacm", "intel_hfi", "mpi_selector",
		"delta_ipoib",
		"opa_stack_dev", "rdma_ndd",
 		"gasnet", "openshmem",
		"mvapich2", "openmpi",
		"delta_mpisrc", 
		"hfi1_uefi",
		"delta_debug", );
my @delta_Components_sles12_sp2 = ( "opa_stack", "intel_hfi", "mpi_selector",
		"delta_ipoib",
		"opa_stack_dev", "rdma_ndd",
 		"gasnet", "openshmem",
		"mvapich2", "openmpi",
		"delta_mpisrc", 
		"hfi1_uefi",
		"delta_debug", );
my @delta_Components_rhel73 = ( "opa_stack", "ibacm", "intel_hfi", "mpi_selector",
		"delta_ipoib",
		"opa_stack_dev", "rdma_ndd",
		"gasnet", "openshmem",
	   	"mvapich2", "openmpi",
	   	"delta_mpisrc", "hfi1_uefi", "delta_debug", );
my @delta_Components_sles12_sp3 = ( "opa_stack", "intel_hfi", "mpi_selector",
		"delta_ipoib",
		"opa_stack_dev", "rdma_ndd",
 		"gasnet", "openshmem",
		"mvapich2", "openmpi",
		"delta_mpisrc", 
		"hfi1_uefi",
		"delta_debug", );
my @delta_Components_rhel74 = ( "opa_stack", "ibacm", "intel_hfi", "mpi_selector",
		"delta_ipoib",
		"opa_stack_dev", "rdma_ndd",
		"gasnet", "openshmem",
	   	"mvapich2", "openmpi",
	   	"delta_mpisrc", "hfi1_uefi", "delta_debug", );
@Components = ( );
# delta_debug must be last

# override some of settings in main_omnipathwrap_delta.pl
sub overrides()
{
	# The component list has slight variations per distro
	if ( "$CUR_VENDOR_VER" eq "ES72" ) {
		@Components = ( @delta_Components_rhel72 );
	} elsif ( "$CUR_VENDOR_VER" eq "ES122" ) {
		@Components = ( @delta_Components_sles12_sp2 );
	} elsif ( "$CUR_VENDOR_VER" eq "ES73" ) {
		@Components = ( @delta_Components_rhel73 );
	} elsif ( "$CUR_VENDOR_VER" eq "ES123" ) {
		@Components = ( @delta_Components_sles12_sp3 );
	} elsif ( "$CUR_VENDOR_VER" eq "ES74" ) {
		@Components = ( @delta_Components_rhel74 );
	} else {
		@Components = ( @delta_Components_other );
	}

	# Sub components for autostart processing
	@SubComponents = ( );

	# TBD remove this concept
	# no WrapperComponent (eg. opaconfig)
	$WrapperComponent = "";

	# set SrcDir for all components to .
	# ofed_delta is a special component only used for the SrcDir of comp.pl
	foreach my $comp ( @Components, "ofed_delta" )
	{
        $ComponentInfo{$comp}{'SrcDir'} = ".";
	}
}
