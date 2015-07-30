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

#
# Converts a message file to a pair of C files (a source file and a header)
# that define macros and data structures for use with the Err module for error
# reporting.
#
# If you encounter any problems with this program please notify Frank Szczerba
# (fszczerba@infiniconsys.com).
#

use strict;
use integer;
use FileHandle;
use File::Basename;

# list of supported languages ( LOG_LANG_ is prepended on output ) must be uppercase
my @languages = qw( ENGLISH );

#### end configuration
my %languages;	# @languages by name
for (my $i = 0; $i < scalar(@languages); $i++) {
	$languages{$languages[$i]} = $i;
}

my $msgFile;
my $msgFileLine;
my $warnCount = 0;
my $errorCount = 0;
my $context;
my $testMode = 0;

sub output {
	my $level = shift;
	my $out = $testMode ? *STDOUT : *STDERR;
	print $out ("$msgFile:$msgFileLine: $level: ",
		  defined $context ? "$context: ": "", @_, "\n");
}

sub warning {
	output "WARNING", @_;
	$warnCount++;
}

sub error {
	output "ERROR", @_;
	$errorCount++;
}

my $defModName;

sub validate_format($);
sub get_names(\%);

#
# file format definition
#
# All sections and attributes should be ucfirst(lower($_)) case
#
my $Format = {
	File => {				# default scope
	},
	Module => {
		Singular => 1,
		Attributes => {
			Name => { Required => 1, Type => "Ident", Min => 1},
			Ucname => { Type => "UcIdent", Calculate => sub { uc $_[0]->{Name}; } },
		},
		Calculate => sub { {Name => $defModName}; },
	},
	Message => {
		Key => "Name",
		Attributes => {
			Name => { Required => 1, Type => "UcIdent" },
			Comment => { },
			Arglist => { Type => "CSV", Max => 6 },
			Unitarg => { Type => "Int", Min => 0, Max => 6, Default => 0, },
			Trapfunc => { Type => "Ident", Default => "LOG_TRAP_NONE" },
			Severity => {
				Recomended => 1,
				Type => "Picklist",
				Transform => sub { $_[0] =~ tr/ /_/;
								   $_[0] =~ s/PARTIAL/ADD_PART/; },
				Set => "Class",
				# Class is set to the RHS below:
				Values => {
					ADD_PART => "LOG_PART",
					ALARM => "LOG_FINAL",
					ERROR => "LOG_FINAL",
					WARNING => "LOG_FINAL",
					FATAL => "LOG_FINAL",
					DUMP => "LOG",
					CONFIG => "LOG",
					PROGRAM_INFO => "LOG",
					PERIODIC_INFO => "LOG",
					DEBUG1_INFO => "LOG",
					DEBUG2_INFO => "LOG",
					DEBUG3_INFO => "LOG",
					DEBUG4_INFO => "LOG",
					DEBUG5_INFO => "LOG",
                                        NOTICE => "LOG_FINAL",
				},
			},
			Description => {
				Required => 1,
				Type => "FmtStr",
				i18n => 1,
			},
			Response => {
				i18n => 1,
				Type => "FmtStr",
				Default => "NULL",
			},
			Correction => {
				i18n => 1,
				Type => "FmtStr",
				Default => "NULL",
			},
		},
	},
	String => {
		Key => "Name",
		Attributes => {
			Name => { Required => 1, Type => "UcIdent" },
			Comment => { },
			Arglist => { Type => "CSV", Max => 6 },
			String => {
				Required => 1,
				Type => "FmtStr",
				i18n => 1,
			},
		},
	}
};

my $File = {};		# the master object
my $object = {};	# temp object

{
	my $scope = 'File';

	#
	# Completes and validates and object and adds it to $File
	#
	sub new_scope($) {
		VALIDATE_OBJ: {
			# validate the object
			#print "new_scope: $scope -> $_[0]\n";
			foreach my $attr (keys(%{$Format->{$scope}{Attributes}})) {
				my $def = $Format->{$scope}{Attributes}{$attr};

				if (not defined $object->{$attr}) {
					if ($def->{Required}) {
						error "Missing $attr attribute";
						last VALIDATE_OBJ;
					}
					else {
						if ($def->{Recomended}) {
							warning "Missing $attr attribute";
						}
						if (defined $def->{Default} or defined $def->{Calculate}) {
							my $default = defined $def->{Default} ? $def->{Default}
																  : &{$def->{Calculate}}($object);
							if ($def->{i18n}) {
								# set default for all languages
								$object->{$attr} = {};
								foreach my $lang (@languages) {
									$object->{$attr}{$lang} = $default;
								}
							}
							else {
								# set default
								$object->{$attr} = $default;
							}
						}
					}
				}
				elsif ($def->{i18n}) {
					# if a message is provided for one language, it must be provided for all
					foreach my $lang (@languages) {
						if (not $object->{$attr}{$lang}) {
							error "$attr: Missing $lang translation";
							last VALIDATE_OBJ;
						}
					}
				}
			}

			if (defined $object->{FmtArgs}) {
				my $numArgs = $object->{FmtArgs};
				if (not defined $object->{Arglist}) {
					error "Missing Arglist ($numArgs args used)";
				}
				else {
					my $listLen = scalar(@{$object->{Arglist}});
					if (defined $Format->{$scope}{Attributes}{Unitarg} and $object->{Unitarg} > $listLen) {
						error "Insufficient arguments ($listLen) for UnitArg ($object->{Unitarg})";
					}
					if ($numArgs > $listLen) {
						error "Insufficient arguments ($listLen) for format ($numArgs args used)";
					}
					elsif ($numArgs < $listLen) {
						warning "Extra arguments ($listLen) for format ($numArgs args used)";
					}
					# check argument names
					get_names(%$object);
				}
			}
			if ($Format->{$scope}{Singular}) {
				$File->{$scope} = $object;
			}
			else {
				if (not exists $File->{$scope}) {
					$File->{$scope} = [];
				}
				elsif ($Format->{$scope}{Key}) {
					my $key = $Format->{$scope}{Key};
					foreach my $exist (@{$File->{$scope}}) {
						if ($exist->{$key} eq $object->{$key}) {
							error "Duplicate $scope definition with $key=$object->{$key}";
							last VALIDATE_OBJ;
						}
					}
				}
				push @{$File->{$scope}}, $object;
			}
		}	# VALIDATE_OBJ
		$object = {};
		$scope = $_[0];
		$context = $scope;
	}

	#
	# Parser
	#
	sub parse_line($) {
		my $line = $_[0];

		#print "$scope: $line\n";
		# parse the line
		LINE: for ($line) {
			# attribute assignement
			/^(\w+)(?:.(\w+))?\s*=\s*(.*)$/ and do {
				my $attr = ucfirst lc $1;
				my $lang;
				my $val = $3;

				if (defined $2) {
					$lang = uc $2;
					if (not defined $languages{$lang}) {
						error "Unknown language '$lang'";
						last;
					}
				}

				if (not exists $Format->{$scope}{Attributes}{$attr}) {
					error "Unknown attribute $attr for section $scope";
					last;
				}

				my $def = $Format->{$scope}{Attributes}{$attr};

				# type and range check
				for ($def->{Type} ? $def->{Type} : "Str") {
					/Int/	and do {
						if ($val !~ /^\d*$/) {
							error "Non-integer value $val for $attr";
							last LINE;
						}
						elsif (defined $def->{Min} and $val < $def->{Min}) {
							error "$attr must be >= $def->{Min}";
							last LINE;
						}
						elsif (defined $def->{Max} and $val > $def->{Max}) {
							error "$attr must be <= $def->{Max}";
							last LINE;
						}
						last;
					};
					/CSV/	and do {
						# convert to a list ref
						$val = [split /\s*,\s*/, $val];

						if (defined $def->{Min} and scalar(@$val) < $def->{Min}) {
							error "$attr must be at least $def->{Min} element(s)";
							last LINE;
						}
						elsif (defined $def->{Max} and scalar(@$val) > $def->{Max}) {
							error "$attr must be no more than $def->{Max} elements";
							last LINE;
						}
						last;
					};
					/CStr|FmtStr/ and do {
						# strip continuation sequences
						$val =~ s/([^\\])(?:"\s*")+/$1/g;
						# and validate
						if ($val !~ /^"(?:[^"\\]*|\\.)*"$/) {
							error "$attr must be a double-quoted C string";
							last LINE;
						}
						# Use Str length check for now, but it will over-count escape sequences
						# and the quotes

						# Fallthrough
					};
					/FmtStr/ and do {
						my $numArgs = validate_format($val);
						$object->{FmtArgs} = $numArgs if (($object->{FmtArgs} || 0) < $numArgs);
						# Fallthrough
					};
					/UcIdent/	and do {
						if ($val !~ /^[A-Z_][A-Z0-9_]*$/) {
							error "$attr must be an uppercase identifier";
							last LINE;
						}
						# Fallthrough
					};
					/Ident/	and do {
						if ($val !~ /^[A-Za-z_][A-Za-z0-9_]*$/) {
							error "$attr must be an identifier";
							last LINE;
						}
						# Fallthrough
					};
					/Ident|Str/	and do {
						if (defined $def->{Min} and length($val) < $def->{Min}) {
							error "$attr must be at least $def->{Min} character(s)";
							last LINE;
						}
						elsif (defined $def->{Max} and length($val) > $def->{Max}) {
							error "$attr must be no more than $def->{Max} characters";
							last LINE;
						}
						last;
					};
					/Picklist/ and do {
						$val = uc $val;
						if (exists $def->{Transform}) {
							&{$def->{Transform}}($val);
						}
						if (not exists $def->{Values}{$val}) {
							error "Invalid $attr";
							last LINE;
						}
						else {
							$object->{$def->{Set}} = $def->{Values}{$val};
						}
						last;
					};
					die "Internal Error, Bad type $def->{Type}\n";
				}

				if ($Format->{$scope}{Attributes}{$attr}{i18n}) {
					if (not defined $lang) {
						error "Missing language";
						last LINE;
					}
					elsif (not exists $object->{$attr}) {
						$object->{$attr} = { $lang => $val };
					}
					else {
						if (exists $object->{$attr}{$lang}) {
							error "Duplicate";
							last LINE;
						}
						$object->{$attr}{$lang} = $val;
					}
				}
				else {
					if (defined $lang) {
						error "Non-internationalized attribute $attr";
						last LINE;
					}
					elsif (exists $object->{$attr}) {
						error "Duplicate attribute $attr";
						last LINE;
					}
					else {
						$object->{$attr} = $val;
						if ($attr eq "Name") {
							$context = "$scope $val";
						}
					}
				}
				last;
			};

			# section header
			/^\[(\w+)\]$/		and do {
				my $new_scope = ucfirst lc $1;

				if ($new_scope eq "Done") {
					# flush
					new_scope("Done");
					# validate sections
					foreach my $check_scope (keys %$Format) {
						#print "check $check_scope: ";
						if (not exists $File->{$check_scope}) {
							#print "Missing ";
							if ($Format->{$check_scope}{Required}) {
								#print "Required\n";
								error "Missing required section $check_scope";
							}
							elsif ($Format->{$check_scope}{Default} or
								   $Format->{$check_scope}{Calculate})
							{
								#print "Defaulting ";
								$object = $Format->{$check_scope}{Default}
												? $Format->{$check_scope}{Default}
												: &{$Format->{$check_scope}{Calculate}};
								#print "Validating ";
								$scope = $check_scope;
								new_scope("Done");
							}
						}
						#print "Done\n";
					}
				}
				elsif ($scope eq "Done") {
					# internal use
					$scope = $new_scope;
				}
				elsif (not exists $Format->{$new_scope}) {
					error "Unknown section $new_scope";
				}
				else {
					if ($Format->{$new_scope}{Singular} and
						(exists $File->{$new_scope} or $scope eq $new_scope))
					{
						error "Only one $new_scope section is permitted";
					}
					new_scope($new_scope);
				}
				last;
			};

			error "Syntax Error";
		}
	}
}

{
	my $f_posarg = '[1-9]\d*\$';
	my $f_flags = '[ #+0\-]';
	my $f_width = "(?:[1-9]\\d*|\\*(?:$f_posarg)?)";
	my $f_precis = "(?:\\.(?:\\d*|\\*(?:$f_posarg)?))";
	my $f_size = '[hq]|(?:ll?)';
	my $f_type = '[cCdiFopsSuxXM]';

			#       $1         $2          $3         $4          $5         $6         $7        $8
	my $fmt = "($f_posarg)?($f_flags*)($f_posarg)?($f_width)?($f_precis)?($f_posarg)?($f_size)?($f_type)";

	#
	# Scan a string and validate printf format specifiers. Returns the number of
	# arguments on success, dies with a descriptive string on error.
	#
	sub validate_format($) {
		my $str = $_[0];
		my $have_pos = 0;
		my $pos_mixed = 0;
		my $numargs = 0;

		$testMode and print "validate_format($str)\n";
		while (length($str) and not $pos_mixed) {
			# strip stuff before the first %
			$str =~ s/^[^%]*//;

			if (length($str)) {
				# is it a valid format?
				if ($str =~ s/^(?:\%\%|\%$fmt)//o) {
					my $numPos = (defined $1) + (defined $3) + (defined $6);
					if ($numPos > 1) {
						error "Multiple positional specifiers '", defined $1 ? $1 : $3,
							  $numPos == 3 ? "', '$3'," : "'",
							  " and '", defined $5 ? $5 : $3, "' in format specifier";
					}
					my $pos = defined $1 ? $1 : defined $3 ? $3 : $6;
					my $flags = $2;
					my $width = $4;
					my $precis = $5;
					my $size = $7;
					my $type = $8;

					# check for repeating flags (not really a problem, but ugly)
					$flags = join('', sort(split //, $flags));
					$testMode and print "Flags are: '$flags'\n";
					if ($flags =~ /(.)\1/) {
						warning "Repeated flag '$1' in format specifier";
					}

					# check for positional and non-positional parameters
					if (defined $pos) {
						my $arg = substr($pos, 0, length($pos) - 1);
						$testMode and print "positional $pos, arg is $arg\n";
						$numargs = $arg if ($arg > $numargs);
						$have_pos = 1;
					}
					else {
						$pos_mixed = $have_pos;
						$numargs++;	# not the right place, but gets the right count
					}
					if (defined $width and $width =~ /^\*($f_posarg)?$/o) {
						if (defined $1) {
							my $arg = substr($1, 0, length($1) - 1);
							$testMode and print "width $width, arg is $arg\n";
							$numargs = $arg if ($arg > $numargs);
							$have_pos = 1;
						}
						else {
							$pos_mixed = $have_pos;
							$numargs++;
						}
					}
					if (defined $precis and $precis =~ /^\.\*($f_posarg)?$/o) {
						if (defined $1) {
							my $arg = substr($1, 0, length($1) - 1);
							$testMode and print "precision $precis, arg is $arg\n";
							$numargs = $arg if ($arg > $numargs);
							$have_pos = 1;
						}
						else {
							$pos_mixed = $have_pos;
							$numargs++;
						}
					}
					if ($have_pos and not defined $pos) {
						$pos_mixed = 1;
					}

					# TODO: validate size-type combos?
				}
				else {
					error "Invalid format specifier at \"$str\"";
					# strip the '%' so we can recover
					$str = substr($str, 1);
				}
			}
		}
		if ($pos_mixed) {
			error "Format mixes positional and non positional parameters";
		}

		return $numargs;
	}

	#
	# Scan a string and return a list of arguments which are IFormattable objects
	#
	sub find_formattable($) {
		my $str = $_[0];
		my $nextarg = 1;
		my %formattable = ();

		while ($str =~ /\G(?:[^\%]|\%\%)*\%$fmt/go)
		{
			my $pos = defined $1 ? $1 : defined $3 ? $3 : $6;
			my $width = $4;
			my $precis = $5;
			my $type = $8;

			if (defined $pos) {
				substr($pos, -1) = '';
				$nextarg = $pos;
			}
			if (defined $width and $width eq '*') {
				$nextarg++;
			}
			if (defined $precis and substr($precis, 1) eq '*') {
				$nextarg++;
			}
			if ($type eq 'F') {
				$formattable{$nextarg}++;
			}
			$nextarg++;
		}

		return sort keys %formattable;
	}

	#
	# Scan a string and return a list of arguments which are pointers
	#
	sub find_pointer($) {
		my $str = $_[0];
		my $nextarg = 1;
		my %pointer = ();

		while ($str =~ /\G(?:[^\%]|\%\%)*\%$fmt/go)
		{
			my $pos = defined $1 ? $1 : defined $3 ? $3 : $6;
			my $width = $4;
			my $precis = $5;
			my $type = $8;

			if (defined $pos) {
				substr($pos, -1) = '';
				$nextarg = $pos;
			}
			if (defined $width and $width eq '*') {
				$nextarg++;
			}
			if (defined $precis and substr($precis, 1) eq '*') {
				$nextarg++;
			}
			if ($type eq 's' or $type eq 'p') {
				$pointer{$nextarg}++;
			}
			$nextarg++;
		}

		return sort keys %pointer;
	}
}

#
# Extract names of arguments from an argument list
#
sub get_names(\%) {
	my $obj = $_[0];
	my @list;
	my %seen = ();

	if (not defined $obj->{Arglist}) {
		return ();
	}

	for (my $i = 1; $i <= scalar(@{$obj->{Arglist}}); $i++) {
		if ($obj->{Arglist}[$i-1] =~ /^([a-zA-Z_]\w+)(?:\s*:.*)?$/) {
			$list[$i-1] = $1;
		}
		else {
			warning "No name given for $obj->{Name} argument $i, defaulting to arg$i";
			$list[$i-1] = "arg$i";
		}
		if ($seen{$list[$i-1]}) {
			error "Duplicate argument name ", $list[$i-1], " for $obj->{Name}";
		}
		$seen{$list[$i-1]} = 1;
	}

	return @list;
}

#
# Find the union of two (or more) lists
#
sub union {
	my %union = ();

	foreach my $i (@_) {
		$union{$i}++;
	}

	return sort keys %union;
}

#
# Main Loop
#

#main()

my $cur_line = "";

if (scalar @ARGV != 1) {
	print "Usage: buildmsgs.pl Module.msg\n";
	print "\nCreates/overwrites Module_Messages.c and Module_Messages.h.\n";
	exit(1);
}

$msgFile = shift;
($defModName,undef,undef) = fileparse($msgFile, '\..*');
$msgFileLine = 0;
my $fileBase = $defModName . "_Messages";

if ($msgFile eq '--test') {
	$testMode = 1;
	while (<>) {
		my $line = $_;
		chomp $line;
		++$msgFileLine;
		my $rslt = validate_format($line);
		print "$rslt arguments\n";
		my @formattable = find_formattable($line);
		my @pointer = find_pointer($line);
		print "IFormattable args: ", join(', ', @formattable), "\n" if (@formattable);
		print "Pointer args: ", join(', ', @pointer), "\n" if (@pointer);
		print "\n";
	}
	exit 0;
}

my $msg_file = new FileHandle "$msgFile" or die "$msgFile: ERROR: $!\n";

while (<$msg_file>) {
	s/((?:".*")*)\s*(?:#.*)?$/$1/;	# strip comments
	s/\s*$//;	# strip trailing whitespace
	if (s/^\t[ \t]*/ /) {	# continuation line
		$cur_line .= $_;
	}
	else {
		if (length($cur_line)) {
			parse_line($cur_line);
		}
		$cur_line = $_;
	}
	++$msgFileLine;
}

# handle last line
if (length($cur_line)) {
	parse_line($cur_line);
}

# flush any in-progress object
parse_line("[Done]");

close $msg_file;

# ensure we have a MODNAME str as the first string
if (not exists $File->{String} or $File->{String}[0]{Name} ne "MODNAME") {
	if (exists $File->{String}) {
		for (my $i = 1; $i < scalar(@{$File->{String}}); $i++) {
			if ($File->{String}[$i]{Name} eq "MODNAME") {
				error "MODNAME must be the first string";
			}
		}
	}
	parse_line("[String]");
	parse_line("Name=MODNAME");
	parse_line("comment=first entry must be 1-6 character package name");
	foreach my $lang (@languages) {
		parse_line("String.$lang=\"$File->{Module}{Name}\"");
	}
	parse_line("[Done]");

	# rotate to the beginning of the list
	unshift @{$File->{String}}, pop @{$File->{String}};
}

# ensure MODNAME str is 1-6 characters
foreach my $lang (@languages) {
	my $str = $File->{String}[0]{String}{$lang};

	# value includes quotes
	if (length $str < 3 or length $str > 9) {
		error "MODNAME string must be 1-6 characters ($lang string $str is ",
			length($str)- 2, " characters";
	}
}

if ($errorCount) {
	die "$msgFile: $warnCount warnings, $errorCount errors\n";
}
elsif ($warnCount) {
	warn "$msgFile: $warnCount warnings\n"
}

#
# generate output
#

my $modName = $File->{Module}{Name};
my $ucModName = $File->{Module}{Ucname};
my $lcModName = lcfirst $modName;
my $mod = "MOD_$ucModName";

my $header = new FileHandle ">$fileBase.h" or die "$msgFile: ERROR: Could not open $fileBase.h for write\n";
my $source = new FileHandle ">$fileBase.c" or die "$msgFile: ERROR: Could not open $fileBase.c for write\n";

#
# Mod_Messages.h
#
my $oldfh = select $header;
print <<"EOF";
#ifndef ${ucModName}_MESSAGES_H_INCLUDED
#define ${ucModName}_MESSAGES_H_INCLUDED

/*!
 \@file    $modName/$fileBase.h
 \@brief   Nationalizable Strings and Messages for $modName Package generated from $msgFile
 */

#include "Gen_Arch.h"
#include "Gen_Macros.h"
#include "Log.h"

EOF

if (exists $File->{Message}) {
	my $need_macros = 0;
	print "/* Messages */\n";
	for (my $i = 0; $i < scalar(@{$File->{Message}}); $i++) {
		printf "#define %s_MSG_%-25s LOG_BUILD_MSGID(%s, %d)\n",
				$ucModName, $File->{Message}->[$i]->{Name}, $mod, $i;
		$need_macros = 1 if (exists $File->{Message}->[$i]->{Severity});
	}
	if ($need_macros) {
		print "\n/* Logging Macros */\n";
		for (my $i = 0; $i < scalar(@{$File->{Message}}); $i++) {
			my $msg = $File->{Message}->[$i];

			next if (not exists $msg->{Severity}); 

			my $msgName = $msg->{Name};
			my @args = get_names(%$msg);
			my @fmt = find_formattable($msg->{Description}{$languages[0]});
			my @ptr = find_pointer($msg->{Description}{$languages[0]});
			if (defined $msg->{Response}) {
				@fmt = union(@fmt, find_formattable($msg->{Response}{$languages[0]}));
				@ptr = union(@ptr, find_pointer($msg->{Response}{$languages[0]}));
			}
			if (defined $msg->{Correction}) {
				@fmt = union(@fmt, find_formattable($msg->{Correction}{$languages[0]}));
				@ptr = union(@ptr, find_pointer($msg->{Correction}{$languages[0]}));
			}
			my @rvArgs = @args;
			foreach my $arg (@fmt) {
				$rvArgs[$arg-1] = 'LOG_FORMATTABLE('. $rvArgs[$arg-1] . ')';
			}
			foreach my $arg (@ptr) {
				$rvArgs[$arg-1] = 'LOG_PTR('. $rvArgs[$arg-1] . ')';
			}
			for (my $arg = scalar(@rvArgs); $arg < 6; $arg++) {
				$rvArgs[$arg] = 0;
			}
			#printf STDOUT "%s: %d, %d\n", $msgName, scalar(@args), scalar(@rvArgs);

			printf "/* %s_MSG_%s", $ucModName, $msgName;
			if (exists $msg->{Comment}) {
				print ": $msg->{Comment}";
			}
			if (exists $msg->{Arglist}) {
				print "\n * Arguments: \n";
				for (my $i = 0; $i < scalar(@{$msg->{Arglist}}); $i++) {
					printf " *\t%s\n", $msg->{Arglist}[$i];
				}
			}
			print " */\n";
			printf "#define %s_%s_%s(%s)\\\n\tLOG_%s(MOD_%s, %s_MSG_%s, %s)\n",
					$ucModName, $msg->{Class}, $msgName, join(', ', @args),
					$msg->{Severity}, $ucModName, $ucModName, $msgName, join(', ', @rvArgs);
		}
	}
}

print "\n/* Constant Strings for use as substitutionals in Messages */\n";
my $need_macros = 0;
for (my $i = 0; $i < scalar(@{$File->{String}}); $i++) {
	printf "#define %s_STR_%-25s LOG_BUILD_STRID(%s, %d)\n",
			$ucModName, $File->{String}->[$i]->{Name}, $mod, $i;
	$need_macros = 1 if (exists $File->{String}->[$i]->{Arglist});
}
if ($need_macros) {
	print "\n/* String Formatting Macros */\n";
	for (my $i = 0; $i < scalar(@{$File->{String}}); $i++) {
		my $str = $File->{String}->[$i];

		next if (not exists $str->{Arglist});

		my @args = get_names(%$str);
		my @fmt = find_formattable($str->{String}{$languages[0]});
		my @ptr = find_pointer($str->{String}{$languages[0]});
		my @rvArgs = @args;
		foreach my $arg (@fmt) {
			$rvArgs[$arg-1] = 'LOG_FORMATTABLE('. $rvArgs[$arg-1] . ')';
		}
		foreach my $arg (@ptr) {
			$rvArgs[$arg-1] = 'LOG_PTR('. $rvArgs[$arg-1] . ')';
		}
		for (my $arg = scalar(@rvArgs); $arg < 6; $arg++) {
			$rvArgs[$arg] = 0;
		}
		printf "/* %s_STR_%s", $ucModName, $str->{Name};
		if (exists $str->{Comment}) {
			print ": $str->{Comment}";
		}
		if (exists $str->{Arglist}) {
			print "\n * Arguments: \n";
			for (my $i = 0; $i < scalar(@{$str->{Arglist}}); $i++) {
				printf " *\t%s\n", $str->{Arglist}[$i];
			}
		}
		print " */\n";
		printf "#define %s_FMT_%s(buffer, %s)\\\n\t", $ucModName, $str->{Name}, join(', ', @args);
		printf "LOG_FORMATBUF(buffer, Log_GetString(%s_STR_%s, (buffer).GetLanguage()), %s)\n",
				$ucModName, $str->{Name}, join(', ', @rvArgs);
	}
}
print "\n";

print <<"EOF";
GEN_EXTERNC(extern void ${modName}_AddMessages();)

#endif /* ${ucModName}_MESSAGES_H_INCLUDED */
EOF

#
# Mod_Messages.c
#
select $source;
print <<"EOF";
/*!
 \@file    $modName/$fileBase.c
 \@brief   Nationalizable Strings and Messages for $modName Package generated from $msgFile
 */

#include "$fileBase.h"

EOF





if (exists $File->{Message}) {   
   # Loop through and add all the trap fun ptr prototypes at the top
   foreach my $msg (@{$File->{Message}}) {
           if(not $msg->{Trapfunc} eq "LOG_TRAP_NONE"){
             printf "extern Log_TrapFunc_t %s;\n", $msg->{Trapfunc};
           }
   }
   print "\n\n";


   print <<"EOF";

/*!
Module specific message table, referenced by Log_MessageId_t.

The entries in the table must provide equivalent argument usage for
all languages.  Argument usage in format strings is used to determine
which arguments may need to be freed
*/

static Log_MessageEntry_t ${lcModName}_messageTable[] =
{
EOF
	foreach my $msg (@{$File->{Message}}) {
		printf "\t{\t%s_MSG_%s,%s\n", $ucModName, $msg->{Name},
									  exists $msg->{Comment} ? "\t/* $msg->{'comment'} */" : '';
		printf "\t\t%d,\t/* argument number which holds unit number, 0 -> none */\n", $msg->{Unitarg};
		printf "\t\t%s,\t/* trap func associated with message */\n", $msg->{Trapfunc};
		print "\t\t/* FUTURE - Binary Data */\n";
		if (exists $msg->{Arglist}) {
			my $args = $msg->{Arglist};
			print "\t\t/* Arguments: \n";
			for (my $i = 0; $i < scalar(@$args); $i++) {
				printf "\t\t *\targ%d -> %s\n", $i + 1, $args->[$i];
			}
			print "\t\t */\n";
		}
		else {
			print "\t\t/* Arguments: None */\n";
		}
		print "\t\t{ /* Language Specific Entries */\n";
		foreach my $lang (@languages) {
			print  "\t\t\t{ /* LOG_LANG_$lang */\n";
			printf "\t\t\t\t/* description */ %s,\n", $msg->{Description}->{$lang};
			printf "\t\t\t\t/* response */    %s,\n", $msg->{Response}->{$lang};
			printf "\t\t\t\t/* correction */  %s,\n", $msg->{Correction}->{$lang};
			print "\t\t\t},\n";
		}
		print "\t\t},\n\t},\n";
	}
	print "};\n\n";
}

print <<"EOF";

/*!
Module specific string table, referenced by Log_StringId_t.
*/

static Log_StringEntry_t ${lcModName}_stringTable[] =
{
EOF
foreach my $str (@{$File->{String}}) {
	printf "\t{\t%s_STR_%s,%s\n", $ucModName, $str->{Name},
								  exists $str->{Comment} ? "\t/* $str->{Comment} */" : '';
	if (defined $str->{Arglist}) {
		my $args = $str->{Arglist};
		print "\t\t/* Arguments: \n";
		for (my $i = 0; $i < scalar(@$args); $i++) {
			printf "\t\t *\targ%d -> %s\n", $i + 1, $args->[$i];
		}
		print "\t\t */\n";
	}
	my $lch = '{';
	foreach my $lang (@languages) {
		printf  "\t\t$lch /* LOG_LANG_$lang */ %s,\n", $str->{String}->{$lang};
		$lch = ' ';
	}
	print "\t\t},\n\t},\n";
}
print "};\n\n";

print <<'EOF';
/*!
Add String table to logging subsystem's tables.

This must be run early in the boot to initialize tables prior to
any logging functions being available.

@return None

@heading Concurrency Considerations
    Must be run once, early in boot

@heading Special Cases and Error Handling
	None

@see Log_AddStringTable
*/

EOF

printf "void\n%s_AddMessages()\n{\n", $modName;
	printf "\tLog_AddStringTable(MOD_%s, %s_stringTable,\n".
		   "\t\t\t\t\t\tGEN_NUMENTRIES(%s_stringTable));\n",
		   $ucModName, $lcModName, $lcModName;
if (exists $File->{Message}) {
	printf "\tLog_AddMessageTable(MOD_%s, %s_messageTable,\n".
		   "\t\t\t\t\t\tGEN_NUMENTRIES(%s_messageTable));\n",
		   $ucModName, $lcModName, $lcModName;
}
print "}\n";

select $oldfh;

close $header;
close $source;
