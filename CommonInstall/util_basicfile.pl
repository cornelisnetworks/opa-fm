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

	system "mkdir -p -m $mode $ROOT$dir";
	system "chown $owner $ROOT$dir";
	system "chgrp $group $ROOT$dir";
}

sub check_dir($)
{
	my($dir) = shift();
	if (! -d "$ROOT$dir" ) 
	{
		#Creating directory 

		make_dir("$dir", "$OWNER", "$GROUP", "ugo=rx,u=rwx");
	}
}

sub isDirectoryEmpty($)
{
	my($dir) = shift();
	my $filecount=0;
	if ( -e $dir and -d $dir )
	{
		$filecount=`ls -la $dir| wc -l`;
		if ( $filecount < 4 )
		{
	    	return 1;
		}       
	}
	return 0;
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
		system "cp -rf $src $ROOT$dest";
		system "chown $owner $ROOT$dest";
		system "chgrp $group $ROOT$dest";
		system "chmod $mode $ROOT$dest";
	}
}

# Copy file and set security context if SELINUX is enabled;
# If SELINUX is disabled, it will behave like copy_file()
sub copy_file_context($$$$$$)
{
	my($src) = shift();
	my($dest) = shift();
	my($owner) = shift();
	my($group) = shift();
	my($mode) = shift();
	my($context) = shift();
	# only copy file if source exists, this keep all those cp errors for litering
	# install for development.

	if ( -e $src)
	{
		# If the destination file exists, somehow the context will not be
		# overwritten using the "--context" option. Remove the file first.
		if ( -e "$ROOT$dest" ) {
			system "rm -rf $ROOT$dest";
		}
		system "cp -rf --context=$context $src $ROOT$dest 2>/dev/null";
		system "chown $owner $ROOT$dest";
		system "chgrp $group $ROOT$dest";
		system "chmod $mode $ROOT$dest";
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
	if (!-d "$ROOT$dest_dir")
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
	system "rm -f $ROOT$file";
}

sub copy_driver_file($$)
{
	copy_file("$_[0]", "$_[1]", "$OWNER", "$GROUP", "ugo=r,u=rw");
}

sub copy_data_file($$)
{
	copy_file("$_[0]", "$_[1]", "$OWNER", "$GROUP", "ugo=r,u=rw");
}

sub copy_all_data_files($$;$)
{
	copy_all_files("$_[0]", "$_[1]", "$OWNER", "$GROUP", "ugo=r,u=rw","$_[2]");
}

sub copy_arlib_file($$)
{
	copy_file("$_[0]", "$_[1]", "$OWNER", "$GROUP", "ugo=r,u=rw");
}

sub copy_systool_file($$)
{
	# Administrator execution only
	copy_file("$_[0]", "$_[1]", "$OWNER", "$GROUP", "ug=rx,u=rwx");
}

sub copy_all_systool_files($$;$)
{
	copy_all_files("$_[0]", "$_[1]", "$OWNER", "$GROUP", "ugo=r,ug=rx,u=rwx", "$_[2]");
}

sub copy_bin_file($$)
{
	# general user execution
	copy_file("$_[0]", "$_[1]", "$OWNER", "$GROUP", "ugo=rx,u=rwx");
}

sub remove_oldarlib($)
{
	my($libbase) = shift();
	if ( "$LIB_DIR" ne "$OLD_LIB_DIR" )
	{
		system "rm -f $ROOT$libbase";
	}
}

#
# Removes a list of files from a directory
# arg0 : target directory
# arg1 : space delimited list of file names.
#
sub remove_file_list($@)
{
	my($dir)=shift();
	my $f;

	foreach $f (@_)
	{
		remove_file("$dir/$f");
	}
}

# remove the files built from source or installed
# (as listed in .files and .dirs)
# $0 = dirname relative to $ROOT
sub remove_installed_files($)
{
	my $dirname = shift();

	# remove files we installed or user compiled, however do not remove
	# any logs or other files the user may have created
	if ( -d "$ROOT/$dirname" ) {
		system "cd $ROOT/$dirname; for dir in `cat .dirs 2>/dev/null`; do ( cd \$dir; make clobber ) >/dev/null 2>&1; done";
		system "cd $ROOT/$dirname; cat .files 2>/dev/null|xargs rm -f 2>/dev/null";
		system "cd $ROOT/$dirname; cat .files 2>/dev/null|sort -r|xargs rmdir 2>/dev/null";
		system "rm -f $ROOT/$dirname/.files 2>/dev/null";
		system "rm -f $ROOT/$dirname/.dirs 2>/dev/null";
		system "rmdir $ROOT/$dirname/ 2>/dev/null";
	}
}

# ===========================================================================
# Shared Libaries
my $RunLdconfig=0;

sub copy_shlib_file($$)
{
	$RunLdconfig=1;
	copy_file("$_[0]", "$_[1]", "$OWNER", "$GROUP", "ugo=rx,u=rw");
}

sub copy_shlib($$$)
{
	my($src) = shift();
	my($libbase) = shift();
	my($version) = shift();

	system "rm -f $ROOT${libbase}.so";
	if ( ! -e $src )
	{
		if ( -e "$src.so.${version}" )
		{
			$src="$src.so.${version}";
		}
		elsif ( -e "$src.so" )
		{
			$src="$src.so";
		}
	}
	if ( -e $src)
	{
		copy_shlib_file("$src", "${libbase}.so.${version}");
		# source dir is not $ROOT so that when boot root, it will work properly
		system "rm -f " . " $ROOT${libbase}.so";
		system "ln -s " . my_basename("${libbase}.so.${version}") . " $ROOT${libbase}.so";
	}
}

sub symlink_usrlocal_shlib($$$)
{
	# a /usr/lib/opa path w/o suffix (eg. a previous dest for copy_shlib)
	my($src) = shift();
	my($libbase) = shift();
	my($version) = shift();

	symlink_usrlocal("${src}.so.${version}", "${libbase}.so.${version}");
	symlink_usrlocal("${src}.so.${version}", "${libbase}.so");
}

sub remove_shlib($)
{
	my($libbase) = shift();
	$RunLdconfig=1;
	system "rm -f $ROOT${libbase}.so $ROOT${libbase}.so.*";
}

sub remove_oldshlib($)
{
	if ( "$LIB_DIR" ne "$OLD_LIB_DIR" )
	{
		remove_shlib("$_[0]");
	}
}

sub check_ldconfig()
{
	if ($RunLdconfig == 1 )
	{
		print_separator;
		print "Updating dynamic linker cache...\n";
		# this is how /etc/rc.sysinit runs ldconfig
		#LogPrint "Updating dynamic linker cache: /sbin/ldconfig -n /lib\n";
		#system "chroot /$ROOT /sbin/ldconfig -n /lib > /dev/null 2>&1";
		LogPrint "Updating dynamic linker cache: /sbin/ldconfig\n";
		system "chroot /$ROOT /sbin/ldconfig > /dev/null 2>&1";
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
