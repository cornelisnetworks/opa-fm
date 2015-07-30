#! perl -w
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

use strict;
use Text::ParseWords;
use IO::File;

my $bsp = shift;
my $product = uc(shift);
my $csvfile = shift;
my $inhfile = shift;
my $outfile = shift;

if (not $outfile) {
	die "usage: $0 bsp product input.csv input.h.in output.h\n";
}

my $CSV = IO::File->new($csvfile, "r") or die "$csvfile: $!\n"; 
my $INH = IO::File->new($inhfile, "r") or die "$inhfile: $!\n"; 
my $OUT = IO::File->new($outfile, "w") or die "$outfile: $!\n"; 

sub split_line {
	$_[0] =~ s/^\s+//;
	$_[0] =~ s/\s+$//;
	$_[0] =~ s/\\n/\n    /g;
	return quotewords(",",0,$_[0]);
}

my @bsps = split_line(scalar <$CSV>);
my $index = 0;
my $comment_idx = scalar @bsps - 1;

for (my $i = 1; $i < $comment_idx; $i++) {
	if ($bsps[$i] eq $bsp) {
		$index = $i;
		last;
	}
}

unless ($index) {
	shift @bsps;
	pop @bsps;
	die "Unknown BSP \"$bsp\". Supported values are (", join(", ", @bsps), ")\n";
}

while (<$INH>) {
	$OUT->print("$_");
	last if (m#\*/#);
}

$OUT->print(<<"EOF"

/**************************************************************
 * EVERYTHING BETWEEN THESE MARKERS IS GENERATED FROM
 * $csvfile
 **************************************************************/

/*
 * DO NOT EDIT BY HAND
 *
 * Use a spreadsheet program (such as kspread) to edit
 * $csvfile
 * then run make to rebuild this file.
 *
 */
EOF
);

while (<$CSV>) {
	my ($flag, $value, $comment) = (split_line($_))[0,$index,$comment_idx];
	if (defined $value) {
		if ($value =~ /(?:PRODUCT_)?$product\{([^}]*)\}/o) {
			# got something like FCIOU{2, 1} or EIOU{}
			$value = $1;
		}
		else {
			$value =~ s/\b[A-Z][A-Z0-9_]*\{[^}]*\}//g;
			$value =~ s/^\s+//;
			$value =~ s/\s+$//;
		}
	}
	if ($comment) {
		$OUT->print("\n/*! $comment\n*/\n");
	}
	else {
		$OUT->print("\n");
	}
	if ($value) {
		$OUT->print("#define $flag\t$value\n");
	}
	else {
		$OUT->print("#undef  $flag\n");
	}
}

$OUT->print(<<"EOF"

/**************************************************************
 * END GENERATED CODE FROM
 * $csvfile
 **************************************************************/

EOF
);

while (<$INH>) {
	$OUT->print("$_");
}
