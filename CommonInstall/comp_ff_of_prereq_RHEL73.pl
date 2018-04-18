#!/usr/bin/perl
## BEGIN_ICS_COPYRIGHT8 ****************************************
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
#Installation Prequisites array for fast fabric
#and of tools component
my @oftools_prereq = (
                    "glibc",
                    "libgcc",
                    "libibmad",
                    "libibumad",
                    "libibverbs",
		    "libhfi1",
                    "libstdc++",
		    "ibacm",
);
$comp_prereq_hash{'oftools_prereq'} = \@oftools_prereq;

my @fastfabric_prereq = (
                    "atlas",
                    "bash",
                    "bc",
                    "expat",
                    "expect",
                    "glibc",
                    "libgcc",
                    "libibmad",
                    "libibumad",
                    "libibverbs",
		    "libhfi1",
                    "libstdc++",
                    "ncurses-libs",
                    "openssl-libs",
                    "perl",
                    "perl-Getopt-Long",
                    "perl-Socket",
                    "rdma",
                    "tcl",
                    "zlib",
		    "qperf",
                    "perftest",
);
$comp_prereq_hash{'fastfabric_prereq'} = \@fastfabric_prereq;

my @opamgt_sdk_prereq = (
			"bash",
			"glibc",
			"libgcc",
			"libibumad",
			"libibumad-devel",
			"libibverbs",
			"libibverbs-devel",
			"libstdc++",
			"openssl",
			"openssl-devel",
			"openssl-libs"
);
$comp_prereq_hash{'opamgt_sdk_prereq'} = \@opamgt_sdk_prereq;
