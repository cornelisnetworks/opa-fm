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

# ==========================================================================
# Fabric Viewer for OFED installation

sub available_qlgc_fv
{
	#my $srcdir=$ComponentInfo{'qlgc_fv'}{'SrcDir'};
	#my $binfile = file_glob("$srcdir/*FV*install.bin");
	#return ( -d "$srcdir" && -e "$binfile" );
	return 0;	# not yet working
}

sub installed_qlgc_fv
{
	my $driver_subdir=$ComponentInfo{'qlgc_fv'}{'DriverSubdir'};
	return 0;	# TBD
}

# only called if installed_qlgc_fv is true
sub installed_version_qlgc_fv
{
	# TBD - can we determine this in another way?
	if ( -e "$ROOT$BASE_DIR/version_qlgc_fv" ) {
		return `cat $ROOT$BASE_DIR/version_qlgc_fv`;
	} else {
		return "";
	}
}

# only called if available_qlgc_fv is true
sub media_version_qlgc_fv
{
	my $srcdir=$ComponentInfo{'qlgc_fv'}{'SrcDir'};
	return `cat "$srcdir/Version"`;
}

sub build_qlgc_fv
{
	my $osver = $_[0];
	my $debug = $_[1];	# enable extra debug of build itself
	my $build_temp = $_[2];	# temp area for use by build
	my $force = $_[3];	# force a rebuild
	my $srcdir=$ComponentInfo{'qlgc_fv'}{'SrcDir'};

	return 0;	# success
}

sub need_reinstall_qlgc_fv($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return "no";
}

sub preinstall_qlgc_fv
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled
	my $srcdir=$ComponentInfo{'qlgc_fv'}{'SrcDir'};

	return 0;	# success
}

sub install_qlgc_fv
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled

	my $srcdir=$ComponentInfo{'qlgc_fv'}{'SrcDir'};
	my $driver_subdir=$ComponentInfo{'qlgc_fv'}{'DriverSubdir'};
	my $rc;

	my $version=media_version_qlgc_fv();
	chomp $version;
	printf("Installing $ComponentInfo{'qlgc_fv'}{'Name'} $version $DBG_FREE...\n");
	LogPrint "Installing $ComponentInfo{'qlgc_fv'}{'Name'} $version $DBG_FREE for $CUR_OS_VER\n";

	# TBD - how to run $srcdir/*FV*install.bin
	# TBD - do we need this or can we figure out in installed_version above
	#check_dir("/opt/iba");
	#copy_systool_file("$srcdir/comp.pl", "/opt/iba/.comp_qlgc_fv.pl");
	#check_dir("$BASE_DIR");
	#copy_data_file("$srcdir/Version", "$BASE_DIR/version_qlgc_fv");

	$ComponentWasInstalled{'qlgc_fv'}=1;
}

sub postinstall_qlgc_fv
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled
}

sub uninstall_qlgc_fv
{
	my $install_list = $_[0];	# total that will be left installed when done
	my $uninstalling_list = $_[1];	# what items are being uninstalled

	my $driver_subdir=$ComponentInfo{'qlgc_fv'}{'DriverSubdir'};
	NormalPrint("Uninstalling $ComponentInfo{'qlgc_fv'}{'Name'}...\n");

	#TBD how to run unininstall
	system("rm -rf $ROOT$BASE_DIR/version_qlgc_fv");
	system "rmdir $ROOT$BASE_DIR 2>/dev/null";  # remove only if empty
	system("rm -rf $ROOT/opt/iba/.comp_qlgc_fv.pl");
	system "rmdir $ROOT/opt/iba 2>/dev/null";  # remove only if empty
	$ComponentWasInstalled{'qlgc_fv'}=0;
}

