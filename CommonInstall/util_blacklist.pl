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
# The functions and constants below assist in editing hotplug and blacklists
# to prevent autoloading of drivers prior to startup scripts

my $HOTPLUG_HARDWARE_DIR = "/etc/sysconfig/hardware";

# if not existing file on OS, use this location and create empty file
# code in add_blacklist depends on pre-existance of file
my $HOTPLUG_BLACKLIST_FILE = "/etc/modprobe.d/opa-blacklist.conf";
my $HOTPLUG_BLACKLIST_PREFIX = "blacklist";
my $HOTPLUG_BLACKLIST_SPACE = " ";
my $HOTPLUG_BLACKLIST_SPACE_WILDCARD = "[ 	]*";

# check if the module is blacklisted
sub is_blacklisted($)
{
	my $module = shift();

	my $file = "${HOTPLUG_BLACKLIST_FILE}";
	my $found;

	if (! -f "$file")
	{
		return 0;
	}

	return ! system("grep -q $module $file");
}

# add to list to prevent automatic hotplug of driver
sub add_blacklist($)
{
	my $module = shift();

	my $file = "${HOTPLUG_BLACKLIST_FILE}";
	my $found;

	if (! -e "$file")
	{
		system("touch $file"); # Blacklist file missing, let's create it.
	} elsif (! -f "$file") {
		return; # Blacklist file exists but is not regular file, abort.
	}
	open (INPUT, "$file");
	open (OUTPUT, ">>$TMP_CONF");

	select (OUTPUT);
	while (($_=<INPUT>))
	{
		if (/^${HOTPLUG_BLACKLIST_PREFIX}${HOTPLUG_BLACKLIST_SPACE_WILDCARD}$module[ 	]*$/)
		{
			$found++;
		} 
		print $_;
	}
	if (!defined($found))
	{
		print "${HOTPLUG_BLACKLIST_PREFIX}${HOTPLUG_BLACKLIST_SPACE}$module\n";
	}
	select(STDOUT);

	close (INPUT);
	close (OUTPUT);
	system "mv $TMP_CONF $file";

	# also remove any hotplug config files which load the driver
	open hwconfig, "ls /$HOTPLUG_HARDWARE_DIR/hwcfg* 2>/dev/null |"
			|| Abort "Unable to open pipe\n";
	while (<hwconfig>) {
		chop;
		if ( ! system("grep -q \"MODULE.*'$module'\" $_ 2>/dev/null") ) {
			system "rm -f $_"
		}
	}
	close hwconfig;
}

sub remove_blacklist($)
{
	my($module) = shift();

	my($file) = "${HOTPLUG_BLACKLIST_FILE}";
	if (! -f "$file")
	{
		return;
	}
	open (INPUT, "$file");
	open (OUTPUT, ">>$TMP_CONF");

	select (OUTPUT);
	while (($_=<INPUT>))
	{
		if (/^${HOTPLUG_BLACKLIST_PREFIX}${HOTPLUG_BLACKLIST_SPACE_WILDCARD}$module[ 	]*$/)
		{
			next;
		} else {
			print $_;
		}
	}
	select(STDOUT);

	close (INPUT);
	close (OUTPUT);
	system "mv $TMP_CONF $file";
	if ( -z "$file" ) {
	   system("rm -f $file"); # blacklist file is empty, remove it.
	}
}

