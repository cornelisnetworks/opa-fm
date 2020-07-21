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

# ==========================================================================
# Installation logging

my $LogLevel = 0;	# 0 = normal/quiet, 1=verbose, 2=very verbose
my $LogFile = "/dev/null";	# set when open_log

sub open_log(;$)
{
	my($logfile) = shift();
	if ( "$logfile" eq "" ) {
		$logfile="/var/log/opa.log";
	}
	$LogFile = $logfile;
	open(LOG_FD, ">>$logfile");
	print LOG_FD "-------------------------------------------------------------------------------\n";
	print LOG_FD my_basename($0) . " $INT_VERSION Run " . `/bin/date`;
	print LOG_FD "$0 @ARGV\n";
	print LOG_FD "Current Dir: " .  getcwd() . "\n";
}

sub close_log()
{
	close LOG_FD;
}

# output to log and screen and die
sub Abort(@)
{
	print(LOG_FD @_);
	die(@_);
}

# output to log and screen
sub NormalPrint(@)
{
	print(@_);
	print(LOG_FD @_);
}

# output to log only
sub LogPrint(@)
{
	print(LOG_FD @_);
}

sub VerbosePrintEnabled()
{
	return ($LogLevel >= 1);
}

sub VerbosePrint(@)
{
	if (VerbosePrintEnabled() ) {
		print(LOG_FD @_);
	}
}
sub DebugPrintEnabled()
{
	return ($LogLevel >= 2);
}

sub DebugPrint(@)
{
	if (DebugPrintEnabled() ) {
		print(LOG_FD @_);
	}
}

# ============================================================================
# Basic input and prompting routines
my $KEY_ESC=27;
my $KEY_CNTL_C=3;
my $KEY_ENTER=13;

my $Default_Prompt=0;	 # use default values at prompts, non-interactive

sub getch()
{
	my $c;
	system("stty -echo raw");
	$c=getc(STDIN);
	system("stty echo -raw");
	return $c;
}

sub HitKeyCont()
{
	if ( $Default_Prompt )
	{
		return;
	}

	print "Hit any key to continue...";
	getch();
	return;
}

sub remove_whitespace($)
{
	my $string=shift();
	chomp($string);
	$string =~ s/^[[:space:]]*//;	# remove leading
	$string =~ s/[[:space:]]*$//;	# remove trailing
	return $string;
}

# return numeric value: minimum <= value <= maximum
sub GetNumericValue($$$$)
{
	my($retval) = 0;

	my($Question) = $_[0];
	my($default) = $_[1];
	my($minvalue) = $_[2];
	my($maxvalue) = $_[3];

        if ( $Default_Prompt ) {
		NormalPrint "$Question -> $default\n";
		if (($default ge $minvalue) && ($default le $maxvalue)) {
			return $default;
		}
		# for invalid default, fall through and prompt
	}

	while (1)
	{
		NormalPrint "$Question [$default]: ";
		chomp($retval = <STDIN>);
		$retval=remove_whitespace($retval);

		if (length($retval) == 0) {
			$retval=$default;
		}
		if (($retval >= $minvalue) && ($retval <= $maxvalue)) {
			LogPrint "$Question -> $retval\n";
			return $retval;
		}
		else {
			NormalPrint "Value Out-of-Range\n";
                }
	}        
}

#return 0 for no, 1 for yes
sub GetYesNo($$)
{
	my($Question) = shift();
	my($default) = shift();

	my($retval) = 1;
	my($answer) = 0;

	if ( $Default_Prompt ) {
		NormalPrint "$Question ->$default\n";
		if ( "$default" eq "y") {
			return 1;
		} elsif ("$default" eq "n") {
			return 0;
		}
		# for invalid default, fall through and prompt
	}

	while ($answer == 0)
	{
		print "$Question [$default]: ";
		chomp($_ = <STDIN>);
		$_=remove_whitespace($_);
		if ("$_" eq "") {
			$_=$default;
		}
		if (/^[Nn]/) 
		{
			LogPrint "$Question -> n\n";
			$retval = 0;
			$answer = 1;
		} elsif (/^[Yy]/ ) {
			LogPrint "$Question -> y\n";
			$retval = 1;
			$answer = 1;
		}
	}        
	return $retval;
}

# we keep answers in text format.  This way we can later enhance the command
# line argument list to allow answers to more complex questions
my %AnswerMemory = ();	# keep track of past answers to yes/no questions

my @AnswerHelp = ();	# help test for each keyword

sub AddAnswerHelp($$)
{
	my($Keyword) = shift();
	my($help) = shift();

	# check for duplicates, skip if already in list
	foreach my $ans (@AnswerHelp)
	{
		if ($ans =~ m/^$Keyword -/)
		{
			return;
		}
	}
	@AnswerHelp = (@AnswerHelp, "$Keyword - $help");
}

sub showAnswerHelp()
{
	if (scalar(@AnswerHelp) != 0) {
	 	printf STDERR "       --answer keyword=value - provide an answer to a question which might\n";
	 	printf STDERR "            occur during the operation.  answers to questions which are not\n";
	 	printf STDERR "            asked are ignored.  Invalid answers will result in prompting\n";
	 	printf STDERR "            for interactive installs or use of the default for non-interactive.\n";
		printf STDERR "            Possible Questions:\n";
		foreach my $help (@AnswerHelp) {
			printf STDERR "              $help\n";
		}
	} else {
	 	printf STDERR "       --answer keyword=value - presently ignored\n";
	}
}

# similar to GetYesNo, except if the question has already been answered
# (or defaulted on the command line), the question is not asked, instead the
# previous answer is provided
sub GetYesNoWithMemory($$$$)
{
	my($Keyword) = shift();	# unique keyword to identify question
	my($remember) = shift(); # remember answer for use next time
	my($Question) = shift(); # the question shown to user
	my($default) = shift();	# the default to use if not already remembered
	my($retval);

	if ( exists $AnswerMemory{"$Keyword"})
	{
		$_ = $AnswerMemory{"$Keyword"};
		if (/^[Nn]/) 
		{
			NormalPrint "$Keyword: $Question -> n\n";
			$retval = 0;
		} elsif (/^[Yy]/ ) {
			NormalPrint "$Keyword: $Question -> y\n";
			$retval = 1;
		} else {
			# invalid answer, now what?  prompt?
			NormalPrint "$Keyword: $Question -> Invalid answer: $_\n";
			delete $AnswerMemory{"$Keyword"};
		}
	}
	if ( ! exists $AnswerMemory{"$Keyword"})
	{
		$retval = GetYesNo($Question, $default);
		if ($remember) {
			my($ans);
			if ( $retval ) {
				$ans = "y";
			} else {
				$ans = "n";
			}
			$AnswerMemory{"$Keyword"} = $ans;
		}
	}
	return $retval;
}

sub set_answer($$)
{
	my($Keyword) = shift();	# unique keyword to identify question
	my($answer) = shift(); # answer to question

	$AnswerMemory{"$Keyword"} = $answer;
}

sub clear_answer($)
{
	my($Keyword) = shift();

	delete $AnswerMemory{"$Keyword"};
}

sub clear_all_answers($)
{
	my $Keyword;

	foreach $Keyword (keys(%AnswerMemory))
	{
		delete $AnswerMemory{"$Keyword"};
	}
}


#return choice
sub GetChoice($$@)
{
	my($Question) = shift();
	my($default) = shift();
	my(@choices) = @_;	# single character choices

	my $c;

	if ( $Default_Prompt ) {
		NormalPrint "$Question -> $default\n";
		foreach $c ( @choices )
		{
			if (my_tolower("$default") eq my_tolower("$c")) {
				return $c;
			}
		}
		# for invalid default, fall through and prompt
	}

	while (1)
	{
		print "$Question [$default]: ";
		chomp($_ = <STDIN>);
		$_=remove_whitespace($_);
		if ("$_" eq "") {
			$_=$default;
		}
		$_ = my_tolower($_);
		foreach $c ( @choices )
		{
			if ("$_" eq my_tolower("$c")) {
				LogPrint "$Question -> $c\n";
				return $c;
			}
		}
	}        
	# NOTREACHED
}

sub print_separator()
{
	print "-------------------------------------------------------------------------------\n";
}

# based on files on install media update DBG_FREE
# if both release and debug are available on media, prompt user
# This routine checks both the location for IbAccess and Open IB FF installs
sub select_debug_release($)
{
	my($srcdir) = shift();
	my $inp;

	# check current directory to determine if Debug available
	if ( (! -d "$srcdir/bin/$ARCH/$CUR_OS_VER/debug") 
	    and (! -d "$srcdir/bin/$ARCH/$CUR_DISTRO_VENDOR.$CUR_VENDOR_VER/lib/debug")
	    and (! -d "$srcdir/bin/$ARCH/$CUR_DISTRO_VENDOR.$CUR_VENDOR_MAJOR_VER/lib/debug") )
	{
		# no choice, only release available
		$DBG_FREE="release";
	}
	elsif ( (! -d "$srcdir/bin/$ARCH/$CUR_OS_VER/release")
			and (! -d "$srcdir/bin/$ARCH/$CUR_DISTRO_VENDOR.$CUR_VENDOR_VER/lib/release")
			and (! -d "$srcdir/bin/$ARCH/$CUR_DISTRO_VENDOR.$CUR_VENDOR_MAJOR_VER/lib/release") )
	{
		# no choice, only debug available
		$DBG_FREE="debug";
	} elsif ( $Default_Prompt ) {
		$DBG_FREE="release";
		printf ("Installing $DBG_FREE Software\n");
	} else {
		printf ("Both Release and Debug versions are available\n\n");
		printf ("Please select which version should be used for install or upgrade operations:\n\n");
		printf ("1) Release Software\n");
		printf ("2) Debug Software\n");

		$inp = getch();

		if ($inp == 1)
		{
			$DBG_FREE="release";
		}
		elsif ($inp == 2)
		{
			$DBG_FREE="debug";
		}
		else 
		{
			printf ("Invalid Choice...\n");
			HitKeyCont;
			return;
		}
	}
}

