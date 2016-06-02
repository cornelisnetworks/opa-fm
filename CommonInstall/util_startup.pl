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

# ===========================================================================
# startup scripts

# on Some Os's this configures startup file dependencies, especially for
# parallel startup optimizations
my $INSSERV_CONF="/etc/insserv.conf";
# where master copy of init scripts are installed
my $INIT_DIR = "/etc/init.d";
my $SYSTEMCTL_EXEC = system("command -v systemctl > /dev/null 2>&1");
# Components and Sub-Components which user has asked to stop
# these could be utilities or drivers
my %StopFacility = ();

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
		system "chroot /$ROOT /sbin/chkconfig $WhichStartup off > /dev/null 2>&1";
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
		system "chroot /$ROOT /sbin/chkconfig --del $WhichStartup > /dev/null 2>&1";
		system "chroot /$ROOT /sbin/chkconfig --add $WhichStartup > /dev/null 2>&1";
		# make sure its enabled now
		system "chroot /$ROOT /sbin/chkconfig $WhichStartup on > /dev/null 2>&1";
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
		open(CHKCONFIG,"chroot /$ROOT /sbin/chkconfig --list $WhichStartup 2> /dev/null |") || Abort "Couldn't open a pipe for chkconfig\n";
		$logoutput = <CHKCONFIG>;
		close(CHKCONFIG);
		# remove startup name from output, this way opamon is not mistaken for "on"
		$logoutput=~s/^$WhichStartup//;
		if (grep /:on/, $logoutput)
		{
			return 1;
		} elsif ( `ls $ROOT/etc/rc.d/rc3.d/S*$WhichStartup 2>/dev/null` ne "" ) {
			# this case is an old install being updated
			return 1;
		} else {
			return 0;
		}
	}
}

# remove startup dependency
sub del_insserv_conf($$)
{
	my($WhichStartup) = shift();	# our startup script
	my($dependent) = shift();		# start up which must preceed it

	if ( -e "$ROOT/$INSSERV_CONF" )
	{
		system "sed -e '/^\$$dependent\[ \\t\]/s/\[ \\t\]+\\+$WhichStartup//' > $TMP_CONF < $ROOT/$INSSERV_CONF";
		system "mv $TMP_CONF $ROOT/$INSSERV_CONF";
		system "chmod 444 $ROOT/$INSSERV_CONF";
	}
}

# add startup dependency
sub add_insserv_conf($$)
{
	my($WhichStartup) = shift();	# our startup script
	my($dependent) = shift();		# startup which must preceed it

	if ( -e "$ROOT/$INSSERV_CONF" )
	{
		del_insserv_conf("$WhichStartup", "$dependent");
		system "sed -e '/^\$$dependent\[ \\t\]/s/\$/ +$WhichStartup/' > $TMP_CONF < $ROOT/$INSSERV_CONF";
		system "mv $TMP_CONF $ROOT/$INSSERV_CONF";
		system "chmod 444 $ROOT/$INSSERV_CONF";
	}
}

# Determine if the specified utility is running.
# Use 'pgrep' to check for a matching utility name.
#
# input:
#	[0] = name of utility as output by 'ps'.
# output:
#	0 == named utility not running (reported by pgrep).
#	1 == named utility is running.

sub IsUtilityRunning($)
{
	my($WhichUtility) = shift();

	my $result;

	if ( ROOT_is_set() )
	{
		return 0;
	}

	$result = system("/usr/bin/pgrep $WhichUtility >/dev/null 2>/dev/null");
	if ($result != 0 )
	{
		return 0;
	} else {
		return 1;
	}
}

# prompts and stops a driver or autostart utility
sub check_stop_facility($$)
{
	my($WhichFacility) = shift();
	my($prompt) = shift();

	if ( ROOT_is_set() )
	{
		return 0;
	}
	if (! exists $StopFacility{$WhichFacility})
	{
		$StopFacility{$WhichFacility} = GetYesNo("Stop $prompt now?", "n");
	}
	return $StopFacility{$WhichFacility};
}

sub stop_utility($$$)
{
	my($UtilityDesc) = shift();
	my($WhichUtility) = shift();	# utility executable in ps
	my($InitScript) = shift();

	my($retval)=0;
	my $prompt;

	if ( ROOT_is_set() )
	{
		return;
	}
	if ( "$UtilityDesc" eq "" )
	{
		$prompt="$WhichUtility";
	} else {
		$prompt="$UtilityDesc ($WhichUtility)";
	}
	if (IsUtilityRunning("$WhichUtility") == 1) 
	{
		if (check_stop_facility("$WhichUtility", "$prompt") == 1)
		{
			if (-e "$INIT_DIR/$InitScript" )
			{
				if($SYSTEMCTL_EXEC eq 0 && ($InitScript eq "opafm" || $InitScript eq "opa"))
				{
					system "systemctl stop $InitScript >/dev/null 2>&1";
				} else {
					system "$INIT_DIR/$InitScript stop";
				}
				$retval=1;
			} else {
				system "/usr/bin/pkill $WhichUtility";
				$retval=1;
			}
			$StopFacility{$WhichUtility} = 0;	# ask again if find still running
		}
	}
	return $retval;
}

sub start_utility($$$$)
{
	my($UtilityDesc) = shift();
	my($UtilityDir) = shift();
	my($WhichUtility) = shift();	# executable in $UtilityDir and found in ps
	my($InitScript) = shift();

	my $prompt;

	if ( ROOT_is_set() )
	{
		return;
	}
	if ( "$UtilityDesc" eq "" )
	{
		$prompt="$WhichUtility";
	} else {
		$prompt="$UtilityDesc ($WhichUtility)";
	}
	if (-e "/$UtilityDir/$WhichUtility" )
	{
		if (IsUtilityRunning("$WhichUtility") == 1) 
		{
			print "$prompt already started...\n";
			if (GetYesNo("Restart $prompt now?", "n") == 1)
			{
				if (-e "$INIT_DIR/$InitScript" )
				{
					if($SYSTEMCTL_EXEC eq 0 && ($InitScript eq "opafm" || $InitScript eq "opa"))
					{
						system "systemctl restart $InitScript >/dev/null 2>&1";
					} else {
						system "$INIT_DIR/$InitScript restart";
					}
				} else {
					system "/usr/bin/pkill $WhichUtility";
				}
			}
		} else {
			if (GetYesNo("Start $prompt now?", "n") == 1)
			{
				if (-e "$INIT_DIR/$InitScript" )
				{
					if($SYSTEMCTL_EXEC eq 0 && ($InitScript eq "opafm" || $InitScript eq "opa"))
					{
						system "systemctl start $InitScript >/dev/null 2>&1";
					} else {
						system "$INIT_DIR/$InitScript start";
					}
				} else {
					system "/usr/bin/pkill $WhichUtility";
				}
			}
		}
	}
}

sub remove_startup($)
{
	my($WhichStartup) = shift();

	# test added to avoid duplicate message
	if ( -e "$ROOT$INIT_DIR/$WhichStartup" )
	{
		print "Removing $WhichStartup init scripts...\n";
	}
	# to be safe, remove and disable it even if missing
	disable_autostart($WhichStartup);
	if($SYSTEMCTL_EXEC ne 0)
	{
		system "chroot /$ROOT /sbin/chkconfig --del $WhichStartup > /dev/null 2>&1";
		system "rm -f $ROOT$INIT_DIR/$WhichStartup";
	}
}

