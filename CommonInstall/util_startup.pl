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

# ===========================================================================
# startup scripts

# on Some Os's this configures startup file dependencies, especially for
# parallel startup optimizations
my $INSSERV_CONF="/etc/insserv.conf";
# where master copy of init scripts are installed
my $INIT_DIR = "/etc/init.d";
my $SYSTEMCTL_EXEC = system("command -v systemctl > /dev/null 2>&1");

sub disable_autostart($)
{
	my($WhichStartup) = shift();

	# disable autostart but leave any kill scripts so stopped on shutdown
	# Note on SLES off removes kill scripts too, on redhat they remain
	if($SYSTEMCTL_EXEC eq 0 && 
		($WhichStartup eq "opafm" || $WhichStartup eq "opa" || $WhichStartup eq "ibacm"))
	{
		system "systemctl disable $WhichStartup >/dev/null 2>&1";
	} else {
		system "/sbin/chkconfig $WhichStartup off > /dev/null 2>&1";
	}
}

sub enable_autostart($)
{
	my($WhichStartup) = shift();

	# cleanup to be safe
	if($SYSTEMCTL_EXEC eq 0 && 
		($WhichStartup eq "opafm" || $WhichStartup eq "opa" || $WhichStartup eq "ibacm"))
	{
		system "systemctl enable $WhichStartup >/dev/null 2>&1";
	} else {
		system "/sbin/chkconfig --del $WhichStartup > /dev/null 2>&1";
		system "/sbin/chkconfig --add $WhichStartup > /dev/null 2>&1";
		# make sure its enabled now
		system "/sbin/chkconfig $WhichStartup on > /dev/null 2>&1";
	}
}

# Is given startup script currently configured to autostart
sub IsAutostart($)
{
	my($WhichStartup) = shift();

	if($SYSTEMCTL_EXEC eq 0 &&
	   ($WhichStartup eq "opafm" || $WhichStartup eq "opa" || $WhichStartup eq "ibacm"))
	{
		my($isEnabled) = `systemctl is-enabled $WhichStartup 2>/dev/null`;
		chomp($isEnabled);
		if($isEnabled eq "disabled" || $isEnabled eq "")
		{
			return 0;
		} else {
			return 1;
		}
	} else {
		my $logoutput;
		open(CHKCONFIG,"/sbin/chkconfig --list $WhichStartup 2> /dev/null |") || Abort "Couldn't open a pipe for chkconfig\n";
		$logoutput = <CHKCONFIG>;
		close(CHKCONFIG);
		# remove startup name from output, this way opamon is not mistaken for "on"
		$logoutput=~s/^$WhichStartup//;
		if (grep /:on/, $logoutput)
		{
			return 1;
		} elsif ( `ls /etc/rc.d/rc3.d/S*$WhichStartup 2>/dev/null` ne "" ||
			`ls /etc/init.d/rc3.d/S*$WhichStartup 2>/dev/null` ne "") {
			# this case is an old install being updated.
			# A SLES OS may have no link from init.d to rc.d. So we check init.d folder as well.
			return 1;
		} else {
			return 0;
		}
	}
}
