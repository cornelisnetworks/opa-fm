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
# Main menus, option handling and version handling for FF for OFED install
#

$MainInstall="opafm";

	# Names of supported install components
	# must be listed in depdency order such that prereqs appear 1st
@Components = ( "opafm" );

	# an additional "special" component which is always installed when
	# we install/upgrade anything and which is only uninstalled when everything
	# else has been uninstalled.  Typically this will be the opaconfig
	# file and related files absolutely required by it (such as wrapper_version)
$WrapperComponent = "opa_config_fm";

# This provides more detailed information about each Component in @Components
# since hashes are not retained in the order specified, we still need
# @Components and @SubComponents to control install and menu order
# Fields are as follows:
#	Name => full name of component/subcomponent for prompts
# 	DefaultInstall => default installation (State_DoNotInstall or State_Install)
#					  used only when available and ! installed
#	SrcDir => directory name within install media for component
#	DriverSubdir => directory within /lib/modules/$CUR_OS_VERSION/ to
#					install driver to.  Set to "" if no drivers in component
#					set to "." if no subdir
# 	PreReq => other components which are prereqs of a given
#				component/subcomponent
#				Need space before and after each component name to
#				facilitate compares so that compares do not mistake names
#				such as mpidev for mpi
#	CoReq =>  other components which are coreqs of a given component
#				Note that CoReqs can also be listed as prereqs to provide
#				install verification as each component is installed,
#				however prereqs should only refer to items earlier in the list
#				Need space before and after each component name to
#				facilitate compares so that compares do not mistake names
#				such as mpidev for mpi
#	Hidden => component should not be shown in menus/prompts
#			  used for hidden PreReq.  Hidden components can't HasStart
#			  nor HasFirmware
#	Disabled =>  components/subcomponents which are disabled from installation
#	HasStart =>  components/subcomponents which have autostart capability
#	DefaultStart =>  should autostart default to Enable (1) or Disable (0)
#				Not needed/ignored unless HasStart=1
# 	StartPreReq => other components/subcomponents which must be autostarted
#				before autostarting this component/subcomponent
#				Not needed/ignored unless HasStart=1
#				Need space before and after each component name to
#				facilitate compares so that compares do not mistake names
#				such as mpidev for mpi
#	StartComponents => components/subcomponents with start for this component
#				if a component has a start script 
#				list the component as a subcomponent of itself
#	HasFirmware =>  components which need HCA firmware update after installed
#
# Note both Components and SubComponents are included in the list below.
# Components require all fields be supplied
# SubComponents only require the following fields:
#	Name, PreReq (should reference only components), HasStart, StartPreReq,
#	DefaultStart
%ComponentInfo = (
	"opa_config_fm" =>	{ Name => "opa_config_fm",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => "", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 0, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => "",
					  StartComponents => [ ],
					},
	"opafm" =>	{ Name => "OPA FM",
					  DefaultInstall => $State_Install,
					  SrcDir => ".", DriverSubdir => "",
					  PreReq => " opa_stack ", CoReq => "",
					  Hidden => 0, Disabled => 0,
					  HasStart => 1, HasFirmware => 0, DefaultStart => 0,
					  StartPreReq => " opa_stack ",
					  StartComponents => [ "opafm" ],
					},
	);
	# translate from startup script name to component/subcomponent name
%StartupComponent = (
				"opafm" => "opafm",
			);
	# has component been loaded since last configured autostart
%ComponentWasInstalled = (
				"opafm" => 0,
			);

# ==========================================================================
# opa_config_fm installation
# This is a special WrapperComponent which only needs:
#	available, install and uninstall
# it cannot have startup scripts, dependencies, prereqs, etc

sub available_opa_config_fm
{
	my $srcdir=$ComponentInfo{'opa_config_fm'}{'SrcDir'};
	return ( -e "$srcdir/INSTALL" );
}

sub install_opa_config_fm
{
	my $install_list = $_[0];	# total that will be installed when done
	my $installing_list = $_[1];	# what items are being installed/reinstalled

	my $srcdir=$ComponentInfo{'opa_config_fm'}{'SrcDir'};
	NormalPrint("Installing $ComponentInfo{'opa_config_fm'}{'Name'}...\n");

	copy_systool_file("$srcdir/INSTALL", "/sbin/opa_config_fm");
	# no need for a version file, opafm will have its own

	$ComponentWasInstalled{'opa_config_fm'}=1;
}

sub uninstall_opa_config_fm
{
	my $install_list = $_[0];	# total that will be left installed when done
	my $uninstalling_list = $_[1];	# what items are being uninstalled

	NormalPrint("Uninstalling $ComponentInfo{'opa_config_fm'}{'Name'}...\n");
	system("rm -rf $ROOT/sbin/opa_config_fm");
	$ComponentWasInstalled{'opa_config_fm'}=0;
}

my $allow_install;
if ( my_basename($0) ne "INSTALL" )
{
	$allow_install=0;
} else {
	$allow_install=1;
}

#
# Check if all basic requirements are ok
#
sub verify_os_ver 
{
	determine_os_version;
	#my $supported_kernel;
	#
	#if (!$allow_install)
	#{
	#	my $catkernels = `cat $ROOT$BASE_DIR/supported_kernels`;
	#	@supported_kernels = split( / +/, $catkernels);
	#} else {
	#	system("echo -n @supported_kernels > ./config/supported_kernels");
	#}
	#
	#foreach my $supported_kernel_path ( @supported_kernels ) 
	#{    
	#	$supported_kernel = my_basename($supported_kernel_path);
	#
	#	$module_dirs[++$#module_dirs] = "/lib/modules/$supported_kernel";
	#}
}

sub Usage
{
	if ( $allow_install ) {
		#printf STDERR "Usage: $0 [-r root] [-v|-vv] [-a|-n|-U|-u|-s|-i comp|-e comp] [-E comp] [-D comp] [--without-depcheck] [--force] [--answer keyword=value]\n";
		# don't document -i and -e (-a and -u should be equivalent)
		printf STDERR "Usage: $0 [-r root] [-v|-vv] [-a|-n|-U|-u|-s|-O|-N] [-E comp] [-D comp] [--without-depcheck] [--force] [--answer keyword=value]\n";
	} else {
		#printf STDERR "Usage: $0 [-r root] [-v|-vv] [-u|-s|-e comp] [-E comp] [-D comp] [--answer keyword=value]\n";
		printf STDERR "Usage: $0 [-r root] [-v|-vv] [-u|-s] [-E comp] [-D comp] [--answer keyword=value]\n";
	}
	printf STDERR "               or\n";
	printf STDERR "Usage: $0 -C\n";
	printf STDERR "               or\n";
	printf STDERR "Usage: $0 -V\n";
	if ( $allow_install ) {
		printf STDERR "       -a - install all ULPs and drivers with default options\n";
		printf STDERR "       -n - install all ULPs and drivers with default options\n";
		printf STDERR "            but with no change to autostart options\n";
		printf STDERR "       -U - upgrade/re-install all presently installed ULPs and drivers with\n";
		printf STDERR "            default options and no change to autostart options\n";
		#printf STDERR "       -i comp - install the given component with default options\n";
		#printf STDERR "            can appear more than once on command line\n";
		printf STDERR "       --without-depcheck - disable check of OS dependencies\n";
		printf STDERR "       --force - force install even if distos don't match\n";
		printf STDERR "                 Use of this option can result in undefined behaviors\n";
		printf STDERR "       -O - Keep current modified rpm config file\n";
		printf STDERR "       -N - Use new default rpm config file\n";
	}
	printf STDERR "       -u - uninstall all ULPs and drivers with default options\n";
	printf STDERR "       -s - enable autostart for all installed drivers\n";
	printf STDERR "       -r - specify alternate root directory, default is /\n";
	#printf STDERR "       -e comp - uninstall the given component with default options\n";
	#printf STDERR "            can appear more than once on command line\n";
	printf STDERR "       -E comp - enable autostart of given component\n";
	printf STDERR "            can appear with -D or more than once on command line\n";
	printf STDERR "       -D comp - disable autostart of given component\n";
	printf STDERR "            can appear with -E or more than once on command line\n";
	printf STDERR "       -v - verbose logging\n";
	printf STDERR "       -vv - very verbose debug logging\n";
	printf STDERR "       -C - output list of supported components\n";
	printf STDERR "       -V - output Version\n";
	showAnswerHelp();
	printf STDERR "       default options retain existing configuration files\n";
	printf STDERR "       supported component names:\n";
	printf STDERR "            ";
	foreach my $comp ( @Components )
	{
		printf STDERR " $comp";
	}
	printf STDERR "\n";
	if (scalar(@SubComponents) > 0) {
		printf STDERR "       additional component names allowed for -E and -D options:\n";
		printf STDERR "            ";
		foreach my $comp ( @SubComponents )
		{
			printf STDERR " $comp";
		}
		printf STDERR "\n";
	}
	exit (2);
}

sub process_args
{
	my $arg;
	my $last_arg;
	my $setroot = 0;
	my $setanswer = 0;
	my $install_opt = 0;
	my $setcomp = 0;
	my $setenabled = 0;
	my $setdisabled = 0;
	my $comp = 0;

	if (scalar @ARGV >= 1) {
		foreach $arg (@ARGV) {
			if ( $setroot ) {
				$ROOT="$arg";
				if (substr($ROOT,0,1) ne "/") {
					printf STDERR "Invalid -r root (must be absolute path): '$ROOT'\n";
					Usage;
				}
				if (! -d $ROOT) {
					printf STDERR "Invalid -r root (directory not found): '$ROOT'\n";
					Usage;
				}
				$setroot=0;
			} elsif ( $setanswer ) {
				my @pair = split /=/,$arg;
				if ( scalar(@pair) != 2 ) {
					printf STDERR "Invalid --answer keyword=value: '$arg'\n";
					Usage;
				}
				set_answer($pair[0], $pair[1]);
				$setanswer=0;
			} elsif ( $setcomp ) {
				foreach $comp ( @Components )
				{
					if ( "$arg" eq "$comp" )
					{
						$Default_Components{$arg} = 1;
						$setcomp=0;
					}
				}
				if ( $setcomp )
				{
					printf STDERR "Invalid component: $arg\n";
					Usage;
				}
			} elsif ( $setenabled ) {
				foreach $comp ( @Components, @SubComponents )
				{
					if ( "$arg" eq "$comp" )
					{
						$Default_EnabledComponents{$arg} = 1;
						$setenabled=0;
					}
				}
				if ( $setenabled )
				{
					printf STDERR "Invalid component: $arg\n";
					Usage;
				}
			} elsif ( $setdisabled ) {
				foreach $comp ( @Components, @SubComponents )
				{
					if ( "$arg" eq "$comp" )
					{
						$Default_DisabledComponents{$arg} = 1;
						$setdisabled=0;
					}
				}
				if ( $setdisabled )
				{
					printf STDERR "Invalid component: $arg\n";
					Usage;
				}
			} elsif ( "$arg" eq "-r" ) {
				$setroot=1;
			} elsif ( "$arg" eq "-v" ) {
				$LogLevel=1;
			} elsif ( "$arg" eq "-vv" ) {
				$LogLevel=2;
			} elsif ( "$arg" eq "--answer" ) {
				$setanswer=1;
			} elsif ( "$arg" eq "-i" ) {
				$Default_CompInstall=1;
				$Default_Prompt=1;
				$setcomp=1;
				if ($install_opt || $Default_CompUninstall) {
					# can't mix -i with other install controls
					printf STDERR "Invalid combination of options: $arg not permitted with previous options\n";
					Usage;
				}
			} elsif ( "$arg" eq "-e" ) {
				$Default_CompUninstall=1;
				$Default_Prompt=1;
				$setcomp=1;
				if ($install_opt || $Default_CompInstall) {
					# can't mix -e with other install controls
					printf STDERR "Invalid combination of options: $arg not permitted with previous options\n";
					Usage;
				}
			} elsif ( "$arg" eq "-E" ) {
				$Default_Autostart=1;
				$Default_EnableAutostart=1;
				$Default_Prompt=1;
				$setenabled=1;
			} elsif ( "$arg" eq "-D" ) {
				$Default_Autostart=1;
				$Default_DisableAutostart=1;
				$Default_Prompt=1;
				$setdisabled=1;
			} elsif ( "$arg" eq "--without-depcheck" ) {
				$rpm_check_dependencies=0;
			} elsif ( "$arg" eq "--force" ) {
				$Force_Install=1;
			} elsif ( "$arg" eq "-C" ) {
				ShowComponents;
				exit(0);
			} elsif ( "$arg" eq "-V" ) {
				printf "$VERSION\n";
				exit(0);
			} else {
				# Install options
				if ( "$arg" eq "-a" ) {
					$Default_Install=1;
				} elsif ( "$arg" eq "-u" ) {
					$Default_Uninstall=1;
				} elsif ( "$arg" eq "-s" ) {
					$Default_Autostart=1;
				} elsif ( "$arg" eq "-n" ) {
					$Default_Install=1;
					$Default_SameAutostart=1;
				} elsif ( "$arg" eq "-U" ) {
					$Default_Upgrade=1;
					$Default_SameAutostart=1;
				} elsif ("$arg" eq "-O") {
					$Default_RpmConfigKeepOld=1;
					$Default_RpmConfigUseNew=0;
				} elsif ("$arg" eq "-N") {
					$Default_RpmConfigKeepOld=0;
					$Default_RpmConfigUseNew=1;
				} else {
					printf STDERR "Invalid option: $arg\n";
					Usage;
				}
				if ($install_opt || $Default_CompInstall
					 || $Default_CompUninstall) {
					# only one of the above install selections
					printf STDERR "Invalid combination of options: $arg not permitted with previous options\n";
					Usage;
				}
				$install_opt=1;
				$Default_Prompt=1;
			}
			$last_arg=$arg;
		}
	}
	if ( $setroot || $setcomp || $setenabled || $setdisabled || $setanswer) {
		printf STDERR "Missing argument for option: $last_arg\n";
		Usage;
	}
	if ( ($Default_Install || $Default_CompInstall || $Default_Upgrade
				  	|| $Force_Install)
		 && ! $allow_install) {
		printf STDERR "Installation options not permitted in this mode\n";
		Usage;
	}
}

my @INSTALL_CHOICES = ();
sub show_menu
{
	my $inp;
	my $max_inp;

	@INSTALL_CHOICES = ();
	if ( $Default_Install ) {
		NormalPrint ("Installing All OPA Software\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 1);
	}
	if ( $Default_CompInstall ) {
		NormalPrint ("Installing Selected OPA Software\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 1);
	}
	if ( $Default_Upgrade ) {
		NormalPrint ("Upgrading/Re-Installing OPA Software\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 1);
	}
	if ($Default_Uninstall ) {
		NormalPrint ("Uninstalling All OPA Software\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 3);
	}
	if ($Default_CompUninstall ) {
		NormalPrint ("Uninstalling Selected OPA Software\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 3);
	}
	if ($Default_Autostart) {
		NormalPrint ("Configuring autostart for Selected installed OPA Drivers\n");
		@INSTALL_CHOICES = ( @INSTALL_CHOICES, 2);
	}
	if (scalar(@INSTALL_CHOICES) > 0) {
		return;
	}
START:	
	system "clear";
	printf ("$BRAND OPA $VERSION Software\n\n");
	if ($allow_install) {
		printf ("   1) Install/Uninstall Software\n");
	} else {
		printf ("   1) Show Installed Software\n");
	}
	printf ("   2) Reconfigure Driver Autostart \n");
	$max_inp=2;
	if (!$allow_install)
	{
		printf ("   3) Uninstall Software\n");
		$max_inp=3;
	}
	printf ("\n   X) Exit\n");


	$inp = getch();      

	if ($inp =~ /[qQ]/ || $inp =~ /[Xx]/ ) {
		@INSTALL_CHOICES = ( 99999 );	# indicates exit, none selected
		return;
	}
	if (ord($inp) == $KEY_ENTER) {           
		goto START;
	}

	if ($inp =~ /[0123456789abcdefABCDEF]/)
	{
		$inp = hex($inp);
	}

	if ($inp < 1 || $inp > $max_inp) 
	{
#		printf ("Invalid choice...Try again\n");
		goto START;
	}
	@INSTALL_CHOICES = ( $inp );
}

process_args;
check_root_user;
open_log("");
verify_os_ver;
verify_modtools;
set_libdir;
if ($allow_install) {
	verify_distrib_files;
}

RESTART:
show_menu;

foreach my $INSTALL_CHOICE ( @INSTALL_CHOICES )
{
	if ($allow_install && $INSTALL_CHOICE == 1) 
	{
		select_debug_release(".");
		show_install_menu(1);
		if (! $Default_Prompt) {
			goto RESTART;
		} else {
			if ($exit_code == 0) {
				print "Done Installing OPA Software.\n"
			} else {
				print "Failed to install all OPA software.\n"
			}
		}
	} 
	elsif ($INSTALL_CHOICE == 1) {
		show_installed(1);
		goto RESTART;
	}
	elsif ($INSTALL_CHOICE == 3)
	{
		show_uninstall_menu(1);
		if (! $Default_Prompt ) {
			goto RESTART;
		} else {
			if ($exit_code == 0) {
				print "Done Uninstalling OPA Software.\n"
			} else {
				print "Failed to uninstall all OPA Software.\n"
			}
		}
	}
	elsif ($INSTALL_CHOICE == 2) 
	{
		reconfig_autostart;
		if (! $Default_Prompt ) {
			goto RESTART;
		} else {
			print "Done OPA Driver Autostart Configuration.\n"
		}
	}
}
close_log;
exit $exit_code;
