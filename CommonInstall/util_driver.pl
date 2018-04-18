#!/usr/bin/perl
# BEGIN_ICS_COPYRIGHT8 ****************************************
# 
# Copyright (c) 2015-2017, Intel Corporation
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

# path to source directories for each set of kernel drivers,
# such as <./bin/$ARCH/*>;
my @supported_kernels;
my @module_dirs = ();

my $RunDepmod=0;	# do we need to run depmod
sub	verify_modtools()
{
	my $look_str;
	my $look_len;
	my $ver;
	my $mod_ver;

	$look_str = "insmod version ";
	$look_len = length ($look_str);
	$ver = `$ROOT/sbin/insmod -V 2>&1 | grep \"$look_str\"`;
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

sub check_depmod()
{
	if ($RunDepmod == 1 )
	{
		print_separator;
		print "Generating module dependencies...\n";
		LogPrint "Generating module dependencies: /sbin/depmod -aev\n";
		system "chroot /$ROOT /sbin/depmod -aev > /dev/null 2>&1";
		$RunDepmod=0;
		return 1;
	}
	return 0;
}

# is the given driver available on the install media
sub available_driver($$)
{
	my($srcdir) = shift();
	my($WhichDriver) = shift();

	$WhichDriver .= $DRIVER_SUFFIX;
	return ( -e "$srcdir/$CUR_OS_VER/$DBG_FREE/$WhichDriver" );
}

# is the given driver installed on this system
sub installed_driver($$)
{
	my($WhichDriver) = shift();
	my($subdir) = shift();
	$WhichDriver .= $DRIVER_SUFFIX;
	return (-e "$ROOT/lib/modules/$CUR_OS_VER/$subdir/$WhichDriver" );
}

# create iba kernel module directory
sub create_driver_dirs($)
{
	my($subdir) = shift();

	my $supported_kernel;
	foreach my $supported_kernel_path ( @supported_kernels ) 
	{    
		$supported_kernel = my_basename($supported_kernel_path);
		if ( -d "$ROOT/lib/modules/$supported_kernel" )
		{
			make_dir("/lib/modules/$supported_kernel/$subdir", "$OWNER", "$GROUP", "ugo=rx,u=rwx");
		}
	}
}

# remove iba kernel module directories
sub remove_driver_dirs($)
{
	my($subdir) = shift();

	my $supported_kernel;
	if ( "$subdir" ne "" ) {
		foreach my $supported_kernel_path ( @supported_kernels ) 
		{    
			$supported_kernel = my_basename($supported_kernel_path);
			if ( -d "$ROOT/lib/modules/$supported_kernel/$subdir" )
			{
				system "rm -rf $ROOT/lib/modules/$supported_kernel/$subdir";
			}
		}
	}

	# remove SDK stuff
	system "rm -rf $ROOT/etc/iba";
}

sub copy_driver($$$)
{
	# TBD put drivers in /lib/modules instead of /lib/modules/iba
	my($WhichDriver) = shift();
	my($subdir) = shift();
	my($verbosity) = shift(); # verbose or silent

	$WhichDriver .= $DRIVER_SUFFIX;

	if ( "$verbosity" ne "silent" ) {
		print ("Copying ${WhichDriver}...\n");
	}
	$RunDepmod=1;
	need_reboot();
	my $supported_kernel;

	foreach my $supported_kernel_path ( @supported_kernels ) 
	{    
		$supported_kernel = my_basename($supported_kernel_path);

		if ( -d "$ROOT/lib/modules/$supported_kernel" )
		{             
			copy_driver_file( "$supported_kernel_path/$DBG_FREE/$WhichDriver",
								"/lib/modules/$supported_kernel/$subdir" );
		}
	}
}

# copy a single driver from srcdir for current OS only
sub copy_driver2($$$$)
{
	my($WhichDriver) = shift();
	my($srcdir) = shift();
	my($subdir) = shift();
	my($verbosity) = shift();	# verbose or silent

	$WhichDriver .= $DRIVER_SUFFIX;

	if ( "$verbosity" ne "silent" ) {
		printf ("Copying ${WhichDriver}...\n");
	}
	$RunDepmod=1;
	need_reboot();
	copy_driver_file( "$srcdir/$WhichDriver", "/lib/modules/$CUR_OS_VER/$subdir" );
}

sub remove_driver($$$)
{
	my($WhichDriver) = shift();
	my($subdir) = shift();
	my($verbosity) = shift();	# verbose or silent

	my $dd;

	$WhichDriver .= $DRIVER_SUFFIX;
	if ( "$verbosity" ne "silent" ) {
		print ("Removing ${WhichDriver}...\n");
	}
	LogPrint ("Removing $subdir/${WhichDriver}...\n");
	$RunDepmod=1;
	need_reboot();
	foreach $dd (@module_dirs)
	{
		if ( -e "$ROOT$dd/$subdir/$WhichDriver" )
		{
			system "rm -rf $ROOT$dd/$subdir/$WhichDriver";  
		}
		if ( "$subdir" ne "" && isDirectoryEmpty("$ROOT$dd/$subdir") )
		{
	    	system "rm -rf $ROOT$dd/$subdir"
		}
	}
}
