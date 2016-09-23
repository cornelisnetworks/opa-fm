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
# IB Development installation

sub available_ibdev
{
	my $srcdir=$ComponentInfo{'ibdev'}{'SrcDir'};
	return ( -d "$srcdir/include/iba" );
}

sub installed_ibdev
{
	return( -d "$ROOT/usr/include/iba" );
}

# only called if installed_ibdev is true
sub installed_version_ibdev
{
	if ( -e "$ROOT$BASE_DIR/version" ) {
		return `cat $ROOT$BASE_DIR/version`;
	} else {
		return "";
	}
}

# only called if available_ibdev is true
sub media_version_ibdev
{
	my $srcdir=$ComponentInfo{'ibdev'}{'SrcDir'};
	return `cat "$srcdir/version"`;
}

sub build_ibdev
{
	my $osver = $_[0];
	my $debug = $_[1];	# enable extra debug of build itself
	my $build_temp = $_[2];	# temp area for use by build
	my $force = $_[3];	# force a rebuild
	return 0;	# success
}

sub need_reinstall_ibdev($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return "no";
}

sub preinstall_ibdev
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled
	return 0;	# success
}

sub install_ibdev
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled

	my $srcdir=$ComponentInfo{'ibdev'}{'SrcDir'};

	my $version=media_version_ibdev();
	chomp $version;
	printf("Installing $ComponentInfo{'ibdev'}{'Name'} $version $DBG_FREE...\n");
	LogPrint "Installing $ComponentInfo{'ibdev'}{'Name'} $version $DBG_FREE for $CUR_OS_VER\n";
	remove_oldarlib("$OLD_LIB_DIR/libcm.a");
	copy_arlib_file("$srcdir/bin/$ARCH/$CUR_OS_VER/lib/$DBG_FREE/libcm.a", "$LIB_DIR/libcm.a");
	remove_oldarlib("$OLD_LIB_DIR/libib_mgt.a");
	copy_arlib_file("$srcdir/bin/$ARCH/$CUR_OS_VER/lib/$DBG_FREE/libib_mgt.a", "$LIB_DIR/libib_mgt.a");
	# Copy all iba include files
	check_dir("/usr/include/iba");
	check_dir("/usr/include/linux/iba");
	copy_all_data_files("$srcdir/include/iba", "/usr/include/iba", "");
	copy_all_data_files("$srcdir/include/linux/iba", "/usr/include/linux/iba", "");
	copy_data_file("$srcdir/include/dsc_export.h", "/usr/include/dsc_export.h");
	copy_data_file("$srcdir/include/ipoib_export.h", "/usr/include/ipoib_export.h");
	vapi_dev("install");
	# remove samples installed in old location
	remove_installed_files "/usr/local/src/iba_samples";
	# Copy all ib sample applications
	check_dir("/usr/lib/opa/src");
	check_dir("/usr/lib/opa/src/iba_samples");
	copy_all_data_files("$srcdir/iba_samples", "/usr/lib/opa/src/iba_samples");
	system "cd $ROOT/usr/lib/opa/src/iba_samples; find . -type d| while read dir; do mv \$dir/Makefile.sample \$dir/Makefile 2>/dev/null; done 2>/dev/null";
	system "cd $srcdir/iba_samples; find . | sed -e 's/Makefile.sample/Makefile/' > $ROOT/usr/lib/opa/src/iba_samples/.files 2>/dev/null";
	system "cd $srcdir/iba_samples; find * -maxdepth 1 -type d > $ROOT/usr/lib/opa/src/iba_samples/.dirs 2>/dev/null";
	$ComponentWasInstalled{'ibdev'}=1;
}

sub postinstall_ibdev
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled
}

sub uninstall_ibdev
{
	my $install_list = $_[0];	# total that will be left installed when done
	my $uninstalling_list = $_[1];	# what items are being uninstalled

	NormalPrint("Uninstalling $ComponentInfo{'ibdev'}{'Name'}...\n");
	remove_oldarlib("$OLD_LIB_DIR/libcm.a");
	system "rm -rf $ROOT$LIB_DIR/libcm.a";
	remove_oldarlib("$OLD_LIB_DIR/libib_mgt.a");
	system "rm -rf $ROOT$LIB_DIR/libib_mgt.a";
	vapi_dev("uninstall");
	system "rm -rf $ROOT/usr/include/iba";
	system "rm -rf $ROOT/usr/include/linux/iba";
	system "rm -rf $ROOT/usr/include/dsc_export.h";
	system "rm -rf $ROOT/usr/include/ipoib_export.h";
	# remove iba_samples we installed or user compiled, however do not remove
	# any logs or other files the user may have created
	remove_installed_files "/usr/lib/opa/src/iba_samples";
	system "rmdir $ROOT/usr/lib/opa/src 2>/dev/null";
	system "rmdir $ROOT/usr/lib/opa 2>/dev/null";
	remove_installed_files "/usr/local/src/iba_samples";
	$ComponentWasInstalled{'ibdev'}=0;
}

