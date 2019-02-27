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
# Basic file/directory install/remove

my $OWNER = "root";
my $GROUP = "root";

# remove any directory prefixes from path
sub my_basename($)
{
	my($path) = shift();

	$path =~ s/.*\/(.*)$/$1/;
	return $path;
}

# check for a file using a glob pattern
# return 1st filename to match pattern
# return "" if no match
sub file_glob($)
{
	my $globpat = shift();	# glob pattern to expand

	my $file;
	# there is a little perl trick here, glob returns a list of matching
	# files.  If we simply assigned glob to a scalar it would retain
	# the rest of the list and return subsequent entries on future glob calls
	# then it would return "" after the last matched name.
	# Hence even if glob matched only 1 name, it would return a 2nd ""
	# value to a future caller.  real subtle problem to figure out.
	
	# the list assignment below allows glob to return a list
	# and all extra list entries are discarded.  Hence this returns the
	# first filename which matches or "" if none are matched
	( $file) = glob("$globpat");

	return "$file";
}

sub make_dir($$$$)
{
	my($dir) = shift();
	my($owner) = shift();
	my($group) = shift();
	my($mode) = shift();

	system "mkdir -p -m $mode $dir";
	system "chown $owner $dir";
	system "chgrp $group $dir";
}

sub check_dir($)
{
	my($dir) = shift();
	if (! -d "$dir" ) 
	{
		#Creating directory 

		make_dir("$dir", "$OWNER", "$GROUP", "ugo=rx,u=rwx");
	}
}

sub copy_file($$$$$)
{
	my($src) = shift();
	my($dest) = shift();
	my($owner) = shift();
	my($group) = shift();
	my($mode) = shift();
	# only copy file if source exists, this keep all those cp errors for litering
	# install for development.

	if ( -e $src)
	{               
		system "cp -rf $src $dest";
		system "chown $owner $dest";
		system "chgrp $group $dest";
		system "chmod $mode $dest";
	}
}

sub symlink_usrlocal($$)
{
	my($src) = shift();
	my($dest) = shift();
}

sub copy_all_files($$$$$;$)
{
	my($src_dir)=shift();
	my($dest_dir)=shift();
	my($owner) = shift();
	my($group) = shift();
	my($mode) = shift();
	my($symlink_dir) = shift();	# optional argument
	my(@files);
	my $f;
	my $d;

	if (!-d "$src_dir" )
	{
	 	return;	
	}
	if (!-d "$dest_dir")
	{
		return;
	}
	@files = glob("$src_dir/*");
	foreach $f (@files)
	{
		$d= my_basename($f);
		if (!-d $f)
		{
			copy_file($f, "$dest_dir/$d", $owner, $group, $mode);
			if ( "$symlink_dir" ne "") {
				symlink_usrlocal("$dest_dir/$d", "$symlink_dir/$d");
			}
		} else {
			check_dir("$dest_dir/$d");
				copy_all_files($f, "$dest_dir/$d", $owner, $group, $mode,"");
			}
		}
	}

sub remove_file($)
{
	my($file) = shift();
	system "rm -f $file";
}

sub copy_data_file($$)
{
	copy_file("$_[0]", "$_[1]", "$OWNER", "$GROUP", "ugo=r,u=rw");
}

sub copy_systool_file($$)
{
	# Administrator execution only
	copy_file("$_[0]", "$_[1]", "$OWNER", "$GROUP", "ug=rx,u=rwx");
}

# (as listed in .files and .dirs)
# $0 = dirname relative to 
sub remove_installed_files($)
{
	my $dirname = shift();

	# remove files we installed or user compiled, however do not remove
	# any logs or other files the user may have created
	if ( -d "/$dirname" ) {
		system "cd /$dirname; for dir in `cat .dirs 2>/dev/null`; do ( cd \$dir; make clobber ) >/dev/null 2>&1; done";
		system "cd /$dirname; cat .files 2>/dev/null|xargs rm -f 2>/dev/null";
		system "cd /$dirname; cat .files 2>/dev/null|sort -r|xargs rmdir 2>/dev/null";
		system "rm -f /$dirname/.files 2>/dev/null";
		system "rm -f /$dirname/.dirs 2>/dev/null";
		system "rmdir /$dirname/ 2>/dev/null";
	}
}

# ===========================================================================
# Shared Libaries
my $RunLdconfig=0;

sub check_ldconfig()
{
	if ($RunLdconfig == 1 )
	{
		print_separator;
		print "Updating dynamic linker cache...\n";
		# this is how /etc/rc.sysinit runs ldconfig
		#LogPrint "Updating dynamic linker cache: /sbin/ldconfig -n /lib\n";
		#system "/sbin/ldconfig -n /lib > /dev/null 2>&1";
		LogPrint "Updating dynamic linker cache: /sbin/ldconfig\n";
		system "/sbin/ldconfig > /dev/null 2>&1";
		$RunLdconfig=0;
		return 1;
	}
	return 0;
}

# ===========================================================================
# Will Reboot be required
my $NeedReboot=0;
sub need_reboot()
{
	$NeedReboot=1;
}

sub check_need_reboot()
{
	if ($NeedReboot == 1 )
	{
		print_separator;
		print BOLD, RED, "A System Reboot is recommended to activate the software changes\n", RESET;
		LogPrint "A System Reboot is recommended to activate the software changes\n";
		return 1;
	}
	return 0;
}
