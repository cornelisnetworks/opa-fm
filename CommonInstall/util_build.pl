#!/usr/bin/perl
# BEGIN_ICS_COPYRIGHT8 ****************************************
#
# Copyright (c) 2015-2020, Intel Corporation
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

# =============================================================================
# The functions and constants below assist in doing builds such as via rpmbuild

# The functions below assume a hash of build options.  A sample is shown below
#my %ofed_kernel_ib_options = (
#	# build option		# arch & kernels supported on
#	"--with-core-mod" => "",	# supported on all
#	"--with-ehca-mod" => "PPC 2\.6\.1[6-9].* 2\.6\.20.* 2\.6\.9-55\.
#							PPC64 2\.6\.1[6-9].* 2\.6\.20.* 2\.6\.9-55\.",
#	"--with-iser-mod" => "ALL 2\.6\.16\..*-.*-.* 2\.6\..*\.el5 2\.6\.9-[3-5].*\.EL.*",
#		# all kernels except 2.6.5*  note .* wildcard need as 2nd kernelpat
#	"--with-rds-mod" => "ALL !2\.6\.5.* .*",
#	"--without-ipoibconf" => "NONE",	# essentially a comment
#			# DAPL not supported for PPC64
#	"--with-dapl" => "!PPC64",	# any kernel for !PPC64
#);

# The arch & kernels specification is quite flexible.
#
# special formats:
# 		"" -> always use option (any arch, any kernels)
# 		"NONE" -> never use option, no remaining lines are processed
#
# typical format is one line per arch, separated by newline (inside string):
# 		"arch1 kernelpat1 kernelpat2
#			arch2 kernelpat3"
# leading spaces and tabs ignored before arch
#
# arch format:
#	exact ARCH value (IA32, PPC, X86_64, etc)
#	ALL - remainder of line applies to all architectures
#			if no matches for kernel are found on this line, continues to later
#			lines
#	!arch - remainder of line applies to all architectures except arch
#			if no matches for kernel are found on this line, continues to later
#			lines.  If no kernels listed, applies to any kernel.
#	if for a given arch line, no kernels are specified, it matches any kernels
#
# kernelpat format:
#	"" -> (eg. no kernelpat specified for arch), option applies to any kernel
#			for given arch
#	 - all patterns are regex style
#		beware: . is a wildcard unless \. used
#	- !kernelpat means not on kernel, unlike !arch this indicates option is
#		not appliable to this specific kernel, but does not affect other kernels

# process osver and $ARCH against the list of allowed kernel/arch for
# a given build option to decide if option allowed for given kernel/arch
sub arch_kernel_is_allowed($$)
{
	my $osver = shift();
	my $archs = shift();

	if ( "$archs" eq "" ) {
		# option applicable to all archs all kernels
		DebugPrint "yes\n";
		return 1;
	} else {
		# newlines separate details of each arch
		my @arch_list = split /\n/, $archs;
		foreach my $archdetails ( @arch_list ) {
			# spaces separate arch and each kernelpattern
			# ignore leading and trailing tabs/spaces
			$archdetails =~ s/^[ 	]*//;	# remove leading
			$archdetails =~ s/[ 	]*$//;	# remove trailing
			# all other whitespace is just separators
			my @archkernels = split /[[:space:]]+/,$archdetails;
			# arch is 1st, rest are kernelpatterns
			my $arch = $archkernels[0];
			shift @archkernels;
			if ( "$arch" eq "NONE" ) {
				DebugPrint "no\n";
				return 0;
			} elsif ( "$ARCH" eq "$arch" || "ALL" eq "$arch"
				|| (substr($arch,0,1) eq "!" && "$ARCH" ne substr($arch,1)) ) {
				if (scalar(@archkernels) == 0) {
					# option applicable to all kernels for arch
					DebugPrint "yes\n";;
					return 1;
				} else {
					# list of kernel patterns specified
					foreach my $kernelpattern ( @archkernels ) {
#print "check $kernelpattern\n";
						if (substr($kernelpattern,0,1) eq "!") {
							$kernelpattern = substr($kernelpattern, 1);
							if ( $osver =~ /^$kernelpattern$/ ) {
								# option not available on this kernel
								DebugPrint "no\n";
								return 0;
							}
						} elsif ( $osver =~ /^$kernelpattern$/ ) {
							# option applicable to this kernel for arch
							DebugPrint "yes\n";
							return 1;
						}
					}
				}
			}
		}
	}
	DebugPrint "no\n";
	return 0;	# not specified
}

# given a build option and a kernel, decide if applicable to ARCH and kernel
# based on lookup in build_options hash
sub build_option_is_allowed($$@)
{
	my $osver = shift();
	my $build_option = shift();
	my(%build_options) = @_;	# list of options and when valid

	return arch_kernel_is_allowed($osver, $build_options{$build_option});
}

# based on $osver and $ARCH decide what build options to use
sub get_build_options($@)
{
	my $osver = shift();
	my(@build_options) = @_;	# list of options and when valid

	my $ret = "";
	my $i;
	for($i=0; $i < scalar(@build_options); $i += 2) {
		my $option = $build_options[$i];
		my $archs = $build_options[$i+1];
		if (arch_kernel_is_allowed($osver, $archs)) {
			DebugPrint "Check $option - yes\n";
			$ret="$ret $option";
		} else {
			DebugPrint "Check $option - no\n";
		}
	}
	return "$ret";
}

# execute build_cmd within srcdir
# return 0 on success, != 0 on failure
sub run_build($$$$)
{
	my $message= shift();	# message for building output
	my $srcdir= shift();
	my $build_cmd= shift();
	my $resfileop= shift();	# "append" or "replace"

	my $rc;
	my $shfileop;
	
	if ( "$resfileop" eq "append" ) {
		$shfileop = ">>";
	} else {
		$shfileop = ">";
	}

	NormalPrint "Building $message...";

	LogPrint "\n  cd $srcdir; $build_cmd $shfileop build.res 2>&1\n";
	$rc=system "cd $srcdir; $build_cmd $shfileop build.res 2>&1";
	if ( $rc != 0 ) {
		NormalPrint "\nERROR - FAILED to build $message, see $srcdir/build.res\n";
		# TBD system("cat $srcdir/build.res >> $LogFile");
	} else {
		NormalPrint " done\n";
	}
	return $rc;
}

