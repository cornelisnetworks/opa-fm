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
#
# [ICS VERSION STRING: unknown]
use strict;

# convert release tag in $1 to a version number
#
sub Usage()
{
	print "Usage: $0 [-i] releasetag\n";
	print "    -i - omit integration build number (I#)\n";
	print "    sample release tag formats:\n";
	print "      release_tag        Version String w/ -i    Version String w/o -i\n";
	print "      R1_0I5             1.0                     1.0.5\n";
	print "      R1_0M1I2           1.0.1                   1.0.1.2\n";
	print "      R1_0B1I2           1.0B1                   1.0B1.2\n";
	print "      R1_0M1A1I3         1.0.1A1                 1.0.1A1.3\n";
	print "      R1_0M0P4I5         1.0.0.4                 1.0.0.4.5\n";
	print "      R1_0B1P4           1.0B1.4                 1.0B1.4\n";
	print "      R0capemayI7        0capemay                0capemay.7\n";
	print "      R0capemayP4I1      0capemay.4              0capemay.4.1\n";
	print "      R0tmri.103114.1522 0tmri.103114.1522       0tmri.103114.1522\n";
	print "      ICS_R2_0I5         2.0                     2.0.5\n";

	exit 2;
}

# TBD "dropI" option
if (scalar @ARGV < 1) {
	Usage;
}
my $releasetag= shift;
my $dropI = 0;
if ("$releasetag" eq "-i") {
	$dropI=1;
	if (scalar @ARGV < 1) {
		Usage;
	}
	$releasetag= shift;
}
	
#print "releasetag=$releasetag\n";

# The algorithm here is purposely copied from ParseReleaseTag and for
# ease of comparison is structured the same as the C version in patch_version.c
# this algorithm must exactly match the one in patch_version.

# remove leading non-digits
$releasetag =~ s/^[^0-9]+//;
#print "releasetag=$releasetag\n";
my $version = "";

my $done=0;
my $processedM=0;
my $processedP=0;
for (my $i = 0; ! $done && $i < length($releasetag); $i++) {
	my $c = substr($releasetag, $i, 1);
	if ($c eq 'I') {
		#print "match I\n";
		# I starts the trailer for the release tag, it is not placed
		# in the version string
		if ($dropI) {
			$done=1;
		} else {
			# replaced with a .

			# if no M tag was processed, must insert a zero placeholder
			if (! $processedM) {
				$version .= '.0';
			}
			# if no P tag was processed, must insert a zero placeholder
			if (! $processedP) {
				$version .= '.0';
			}
			$version .= '.';
		}
	} elsif ($c eq '_') {
		#print "match _\n";
		# replaced with a . 
		$version .= '.';
	} elsif ($c eq 'M') {
		#print "match M\n";
		# replaced with a .
		$version .= '.';
		$processedM = 1;
	} elsif ($c eq 'P') {
		#print "match P\n";
		# replaced with a .
		# if no M tag was processed, must insert a zero placeholder
		if (! $processedM) {
			$version .= '.0';
			$processedM = 1;
		}
		$version .= '.';
		$processedP = 1;
	} else {
		#print "no match\n";
		$version .= "$c";
	}
	#printf "c=$c version=$version\n";
}
# version strings must start with a digit
if ($version !~ /^[0-9]/) {
	printf STDERR "\nInvalid release tag format yields: $version\n";
	Usage;
}

print "$version\n";
exit 0;
