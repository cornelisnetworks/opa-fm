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

# ============================================================================
# Driver file management

my $RunDepmod=0;	# do we need to run depmod
sub	verify_modtools()
{
	my $look_str;
	my $look_len;
	my $ver;
	my $mod_ver;

	$look_str = "insmod version ";
	$look_len = length ($look_str);
	$ver = `/sbin/insmod -V 2>&1 | grep \"$look_str\"`;
	chomp $ver;
	$mod_ver = substr ($ver, $look_len - 1, length($ver)-$look_len+1);
	$_ = $mod_ver;
	/2\.[34]\.[0-9]+/;
	#$MOD_UTILS_REV;
	if ($& eq "")
	{
		NormalPrint "Unable to proceed, modutils tool old: $mod_ver\n";
		Abort "Install newer version of modutils 2.3.15 or greater" ;
	}
}

# the following two routines are used for STL-14186. hfi2 rpm doesn't call depmod on uninstall, so we need to explicitly
# call it when we uninstall opa_stack. After we resolve the issue, we shall remove the following 2 routines
sub clear_run_depmod()
{
	$RunDepmod=0;
}

sub set_run_depmod()
{
	$RunDepmod=1;
}

sub check_depmod()
{
	if ($RunDepmod == 1 )
	{
		print_separator;
		print "Generating module dependencies...\n";
		LogPrint "Generating module dependencies: /sbin/depmod -aev\n";
		system "/sbin/depmod -aev > /dev/null 2>&1";
		$RunDepmod=0;
		return 1;
	}
	return 0;
}
