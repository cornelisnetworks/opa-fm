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
# IB Boot installation

sub available_ibboot
{
	my $srcdir=$ComponentInfo{'ibboot'}{'SrcDir'};
	return ( -e "$srcdir/Firmware/InfiniServCC.A1.release.boot" );
}

sub installed_ibboot
{
	return( -e "$ROOT/$BASE_DIR/mt23108/InfiniServCC.A1.boot.bin" );
}

# only called if installed_ibboot is true
sub installed_version_ibboot
{
	if ( -e "$ROOT$BASE_DIR/version" ) {
		return `cat $ROOT$BASE_DIR/version`;
	} else {
		return "";
	}
}

# only called if available_ibboot is true
sub media_version_ibboot
{
	my $srcdir=$ComponentInfo{'ibboot'}{'SrcDir'};
	return `cat "$srcdir/version"`;
}

sub build_ibboot
{
	my $osver = $_[0];
	my $debug = $_[1];	# enable extra debug of build itself
	my $build_temp = $_[2];	# temp area for use by build
	my $force = $_[3];	# force a rebuild
	return 0;	# success
}

sub need_reinstall_ibboot($$)
{
	my $install_list = shift();	# total that will be installed when done
	my $installing_list = shift();	# what items are being installed/reinstalled

	return "no";
}

sub preinstall_ibboot
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled
	return 0;	# success
}

sub install_ibboot
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled

	my $srcdir=$ComponentInfo{'ibboot'}{'SrcDir'};

	my $version=media_version_ibboot();
	chomp $version;
	printf("Installing $ComponentInfo{'ibboot'}{'Name'} $version $DBG_FREE...\n");
	LogPrint "Installing $ComponentInfo{'ibboot'}{'Name'} $version $DBG_FREE for $CUR_OS_VER\n";

	# only selected HCAs have IB Boot enabled firmware images
	# iba is a prereq so basic firmware files are already installed
	# production firmware for cougar 
	copy_boot_firmware($srcdir, "InfiniServC.A1", "mt23108");
	# production firmware for cougarcub
	copy_boot_firmware($srcdir, "InfiniServCC.A1", "mt23108");

	# production firmware for Arbel Lion Cub
	copy_boot_firmware($srcdir, "InfiniServEx.A0", "mt25208");
	copy_boot_firmware($srcdir, "InfiniServExC.A0", "mt25208");
	copy_boot_firmware($srcdir, "InfiniServEx_D.A0", "mt25208");
	copy_boot_firmware($srcdir, "InfiniServEx_D.20", "mt25208");

	$ComponentWasInstalled{'ibboot'}=1;
}

sub postinstall_ibboot
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled
}

sub uninstall_ibboot
{
	my $install_list = $_[0];	# total that will be left installed when done
	my $uninstalling_list = $_[1];	# what items are being uninstalled

	NormalPrint("Uninstalling $ComponentInfo{'ibboot'}{'Name'}...\n");

	# production firmware for cougar 
	remove_boot_firmware("InfiniServC.A1", "mt23108");
	# production firmware for cougarcub
	remove_boot_firmware("InfiniServCC.A1", "mt23108");

	# production firmware for Arbel Lion Cub
	remove_boot_firmware("InfiniServEx.A0", "mt25208");
	remove_boot_firmware("InfiniServExC.A0", "mt25208");
	remove_boot_firmware("InfiniServEx_D.A0", "mt25208");
	remove_boot_firmware("InfiniServEx_D.20", "mt25208");

	$ComponentWasInstalled{'ibboot'}=0;
}

