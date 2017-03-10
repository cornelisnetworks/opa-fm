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

#[ICS VERSION STRING: unknown]
use strict;
use File::Basename;

#############################################################################
##
##    MPI installation generic functions

# these functions can be used by the MPI specific install functions

# installs PSM based MPI component.  also handles reinstall on top of
# existing installation and upgrade.
sub install_generic_mpi
{
    my $install_list = $_[0];
    my $installing_list = $_[1];
    my $mpiname = $_[2];
    my $compiler = $_[3];
    my $mpifullname = "$mpiname"."_$compiler"."_hfi";
    my $srcdir = $ComponentInfo{$mpifullname}{'SrcDir'};
    my $version = eval "media_version_$mpifullname()";
    
    printf ("Installing $ComponentInfo{$mpifullname}{'Name'} $version...\n");
    LogPrint ("Installing $ComponentInfo{$mpifullname}{'Name'} $version for $CUR_OS_VER\n");
    # make sure any old potentially custom built versions of mpi are uninstalled
    my @list = ( "$mpifullname", "mpitests_$mpifullname" );
    rpm_uninstall_list2("any", "--nodeps", 'silent', @list);
    
    my $rpmfile = rpm_resolve("$srcdir", "any", "$mpifullname");
    if ( "$rpmfile" ne "" && -e "$rpmfile" ) {
	my $mpich_prefix= "/usr/mpi/$compiler/$mpiname-"
	    . rpm_query_attr($rpmfile, "VERSION") . "-hfi";
	if ( -d "$mpich_prefix" ) {
	    if (GetYesNo ("Remove $mpich_prefix directory?", "y")) {
		LogPrint "rm -rf $mpich_prefix\n";
		system("rm -rf $mpich_prefix");
	    }
	}
    }
    # enable this code if mpitests is missing for some compilers or MPIs
    #my $mpitests_rpmfile = rpm_resolve("$srcdir/OtherMPIs", "any", "mpitests_$mpifullname");
    #if ( "$mpitests_rpmfile" ne "" && -e "$mpitests_rpmfile" ) {
        rpm_install_list_with_options ("$srcdir", "user", " -U --nodeps ", @list);
    #} else {
    #    rpm_install("$srcdir/OtherMPIs", "user", "$mpifullname");
    #}
    check_dir ("/opt/iba");
    copy_systool_file ("$srcdir/comp.pl", "/usr/lib/opa/.comp_$mpifullname.pl");
    
    $ComponentWasInstalled{$mpifullname} = 1;
}

sub installed_generic_mpi
{
    my $mpiname = $_[0];
    my $compiler = $_[1];
    my $mpifullname = "$mpiname"."_$compiler"."_hfi";

    return ( -e "$ROOT/usr/lib/opa/.comp_$mpifullname.pl"
		   	|| rpm_is_installed ($mpifullname, "user") );
}

sub uninstall_generic_mpi
{
    my $install_list = $_[0];
    my $installing_list = $_[1];
    my $mpiname = $_[2];
    my $compiler = $_[3];
    my $mpifullname = "$mpiname"."_$compiler"."_hfi";
    my $rc;
    my $top;
    
    NormalPrint ("Uninstalling $ComponentInfo{$mpifullname}{'Name'}...\n");
    if (lc($CUR_DISTRO_VENDOR) eq "redhat" &&
	($CUR_VENDOR_VER eq "ES6" || $CUR_VENDOR_VER eq "ES6.1")) {
	$top = rpm_query_attr_pkg("$mpifullname", "INSTPREFIXES");
    } else {
	$top = rpm_query_attr_pkg("$mpifullname", "INSTALLPREFIX");
    }
    if ($top eq "" || $top =~ /is not installed/) {
	$top = undef;
    } else {
	$top = `dirname $top`;
	chomp $top;
    }
    
    # uninstall tests in case built by do_build
    $rc = rpm_uninstall ("mpitests_$mpifullname", "user", "", "verbose");
    $rc = rpm_uninstall ($mpifullname, "user", "", "verbose");

    if ( -d $top ) {
	my @files = glob("$top/*");
	my $num = scalar (@files);
	if ( $num == 0 ) {
	    system ("rm -rf $top");
	}
    }
    
    system ("rm -rf $ROOT/usr/lib/opa/.comp_$mpifullname.pl");
    system ("rmdir $ROOT/usr/lib/opa 2>/dev/null"); # remove only if empty
    $ComponentWasInstalled{$mpifullname} = 0;
}

#############################################################################
##
##    OpenMPI GCC

# immediately stops or starts given component, only
# called if HasStart
sub start_openmpi_gcc_hfi
{
}

# is component X available on the install media (use of this
# allows for optional components in packaging or limited availability if a
# component isn't available on some OS/CPU combos)
sub stop_openmpi_gcc_hfi
{
}

# is component X available on the install media (use of this
# allows for optional components in packaging or limited availability if a
# component isn't available on some OS/CPU combos)
sub available_openmpi_gcc_hfi
{
    my $srcdir = $ComponentInfo{'openmpi_gcc_hfi'}{'SrcDir'};
    return rpm_exists ("$srcdir", "user", "openmpi_gcc_hfi");
}

# is component X presently installed on the system.  This is
# a quick check, not a "verify"
sub installed_openmpi_gcc_hfi
{
    return  installed_generic_mpi("openmpi", "gcc");
}

# what is the version installed on system.  Only
# called if installed_X is true.  versions are short strings displayed and
# logged, no operations are done (eg. only compare for equality)
sub installed_version_openmpi_gcc_hfi
{
    return rpm_query_version_release_pkg ("openmpi_gcc_hfi");
}

# only called if available_X.  Indicates version on
# media.  Will be compared with installed_version_X to determine if
# present installation is up to date.  Should return exact same format for
# version string so comparison of equality is possible.
sub media_version_openmpi_gcc_hfi
{
    my $srcdir = $ComponentInfo{'openmpi_gcc_hfi'}{'SrcDir'};
    my $rpm = rpm_resolve ("$srcdir", "user", "openmpi_gcc_hfi");
    return rpm_query_version_release ($rpm);
}

# used to build/rebuild component on local system (if
# supported).  We support this for many items in comp_ofed.pl
# Other components (like SM) are
# not available in source and hence do not support this and simply
# implement a noop.
sub build_openmpi_gcc_hfi
{
    my $osver = $_[0];
    my $debug = $_[1];
    my $build_temp = $_[2];
    my $force = $_[3];

    return 0;
}

# does this need to be reinstalled.  Mainly used for
# ofed due to subtle changes such as install prefix or kernel options
# which may force a reinstall.  You'll find this is a noop in most others.
sub need_reinstall_openmpi_gcc_hfi
{
    my $install_list = shift ();
    my $installing_list = shift ();

    return "no";
}

# called for all components before they are installed.  Use to verify OS
# has proper dependent rpms installed.
sub check_os_prereqs_openmpi_gcc_hfi
{
	return rpm_check_os_prereqs("openmpi_gcc_hfi", "user");
}

# called for all components before they are installed.  Use
# to build things if needed, etc.
sub preinstall_openmpi_gcc_hfi
{
    my $install_list = $_[0];
    my $installing_list = $_[1];

    my $full = "";
    my $rc;

    return 0;
}

# installs component.  also handles reinstall on top of
# existing installation and upgrade.
sub install_openmpi_gcc_hfi
{
    install_generic_mpi("$_[0]", "$_[1]", "openmpi", "gcc");
}

# called after all components are installed.
sub postinstall_openmpi_gcc_hfi
{
    my $install_list = $_[0];     # total that will be installed when done
    my $installing_list = $_[1];  # what items are being installed/reinstalled
}

# uninstalls component.  May be called even if component is
# partially or not installed at all in which case should do its best to
# get rid or what might remain of component from a previously aborted
# uninstall or failed install
sub uninstall_openmpi_gcc_hfi
{
    uninstall_generic_mpi("$_[0]", "$_[1]", "openmpi", "gcc");
}

#############################################################################
##
##    OpenMPI Intel

# immediately stops or starts given component, only
# called if HasStart
sub start_openmpi_intel_hfi
{
}

# is component X available on the install media (use of this
# allows for optional components in packaging or limited availability if a
# component isn't available on some OS/CPU combos)
sub stop_openmpi_intel_hfi
{
}

# is component X available on the install media (use of this
# allows for optional components in packaging or limited availability if a
# component isn't available on some OS/CPU combos)
sub available_openmpi_intel_hfi
{
    my $srcdir = $ComponentInfo{'openmpi_intel_hfi'}{'SrcDir'};
    return rpm_exists ("$srcdir", "user", "openmpi_intel_hfi");
}

# is component X presently installed on the system.  This is
# a quick check, not a "verify"
sub installed_openmpi_intel_hfi
{
    return  installed_generic_mpi("openmpi", "intel");
}

# what is the version installed on system.  Only
# called if installed_X is true.  versions are short strings displayed and
# logged, no operations are done (eg. only compare for equality)
sub installed_version_openmpi_intel_hfi
{
    return rpm_query_version_release_pkg ("openmpi_intel_hfi");
}

# only called if available_X.  Indicates version on
# media.  Will be compared with installed_version_X to determine if
# present installation is up to date.  Should return exact same format for
# version string so comparison of equality is possible.
sub media_version_openmpi_intel_hfi
{
    my $srcdir = $ComponentInfo{'openmpi_intel_hfi'}{'SrcDir'};
    my $rpm = rpm_resolve ("$srcdir", "user", "openmpi_intel_hfi");
    return rpm_query_version_release ($rpm);
}

# used to build/rebuild component on local system (if
# supported).  We support this for many items in comp_ofed.pl
# Other components (like SM) are
# not available in source and hence do not support this and simply
# implement a noop.
sub build_openmpi_intel_hfi
{
    my $osver = $_[0];
    my $debug = $_[1];
    my $build_temp = $_[2];
    my $force = $_[3];

    return 0;
}

# does this need to be reinstalled.  Mainly used for
# ofed due to subtle changes such as install prefix or kernel options
# which may force a reinstall.  You'll find this is a noop in most others.
sub need_reinstall_openmpi_intel_hfi
{
    my $install_list = shift ();
    my $installing_list = shift ();

    return "no";
}

# called for all components before they are installed.  Use to verify OS
# has proper dependent rpms installed.
sub check_os_prereqs_openmpi_intel_hfi
{
	return rpm_check_os_prereqs("openmpi_intel_hfi", "user");
}

# called for all components before they are installed.  Use
# to build things if needed, etc.
sub preinstall_openmpi_intel_hfi
{
    my $install_list = $_[0];
    my $installing_list = $_[1];

    my $full = "";
    my $rc;

    return 0;
}

# installs component.  also handles reinstall on top of
# existing installation and upgrade.
sub install_openmpi_intel_hfi
{
    install_generic_mpi("$_[0]", "$_[1]", "openmpi", "intel");
}

# called after all components are installed.
sub postinstall_openmpi_intel_hfi
{
    my $install_list = $_[0];     # total that will be installed when done
    my $installing_list = $_[1];  # what items are being installed/reinstalled
}

# uninstalls component.  May be called even if component is
# partially or not installed at all in which case should do its best to
# get rid or what might remain of component from a previously aborted
# uninstall or failed install
sub uninstall_openmpi_intel_hfi
{
    uninstall_generic_mpi("$_[0]", "$_[1]", "openmpi", "intel");
}

#############################################################################
##
##    OpenMPI PGI

# immediately stops or starts given component, only
# called if HasStart
sub start_openmpi_pgi_hfi
{
}

# is component X available on the install media (use of this
# allows for optional components in packaging or limited availability if a
# component isn't available on some OS/CPU combos)
sub stop_openmpi_pgi_hfi
{
}

# is component X available on the install media (use of this
# allows for optional components in packaging or limited availability if a
# component isn't available on some OS/CPU combos)
sub available_openmpi_pgi_hfi
{
    my $srcdir = $ComponentInfo{'openmpi_pgi_hfi'}{'SrcDir'};
    return rpm_exists ("$srcdir", "user", "openmpi_pgi_hfi");
}

# is component X presently installed on the system.  This is
# a quick check, not a "verify"
sub installed_openmpi_pgi_hfi
{
    return  installed_generic_mpi("openmpi", "pgi");
}

# what is the version installed on system.  Only
# called if installed_X is true.  versions are short strings displayed and
# logged, no operations are done (eg. only compare for equality)
sub installed_version_openmpi_pgi_hfi
{
    return rpm_query_version_release_pkg ("openmpi_pgi_hfi");
}

# only called if available_X.  Indicates version on
# media.  Will be compared with installed_version_X to determine if
# present installation is up to date.  Should return exact same format for
# version string so comparison of equality is possible.
sub media_version_openmpi_pgi_hfi
{
    my $srcdir = $ComponentInfo{'openmpi_pgi_hfi'}{'SrcDir'};
    my $rpm = rpm_resolve ("$srcdir", "user", "openmpi_pgi_hfi");
    return rpm_query_version_release ($rpm);
}

# used to build/rebuild component on local system (if
# supported).  We support this for many items in comp_ofed.pl
# Other components (like SM) are
# not available in source and hence do not support this and simply
# implement a noop.
sub build_openmpi_pgi_hfi
{
    my $osver = $_[0];
    my $debug = $_[1];
    my $build_temp = $_[2];
    my $force = $_[3];

    return 0;
}

# does this need to be reinstalled.  Mainly used for
# ofed due to subtle changes such as install prefix or kernel options
# which may force a reinstall.  You'll find this is a noop in most others.
sub need_reinstall_openmpi_pgi_hfi
{
    my $install_list = shift ();
    my $installing_list = shift ();

    return "no";
}

# called for all components before they are installed.  Use to verify OS
# has proper dependent rpms installed.
sub check_os_prereqs_openmpi_pgi_hfi
{
	return rpm_check_os_prereqs("openmpi_pgi_hfi", "user");
}

# called for all components before they are installed.  Use
# to build things if needed, etc.
sub preinstall_openmpi_pgi_hfi
{
    my $install_list = $_[0];
    my $installing_list = $_[1];

    my $full = "";
    my $rc;

    return 0;
}

# installs component.  also handles reinstall on top of
# existing installation and upgrade.
sub install_openmpi_pgi_hfi
{
	install_generic_mpi("$_[0]", "$_[1]", "openmpi", "pgi");
}

# called after all components are installed.
sub postinstall_openmpi_pgi_hfi
{
    my $install_list = $_[0];     # total that will be installed when done
    my $installing_list = $_[1];  # what items are being installed/reinstalled
}

# uninstalls component.  May be called even if component is
# partially or not installed at all in which case should do its best to
# get rid or what might remain of component from a previously aborted
# uninstall or failed install
sub uninstall_openmpi_pgi_hfi
{
    uninstall_generic_mpi("$_[0]", "$_[1]", "openmpi", "pgi");
}

#############################################################################
##
##    MVAPICH2 GCC

# immediately stops or starts given component, only
# called if HasStart
sub start_mvapich2_gcc_hfi
{
}

# is component X available on the install media (use of this
# allows for optional components in packaging or limited availability if a
# component isn't available on some OS/CPU combos)
sub stop_mvapich2_gcc_hfi
{
}

# is component X available on the install media (use of this
# allows for optional components in packaging or limited availability if a
# component isn't available on some OS/CPU combos)
sub available_mvapich2_gcc_hfi
{
    my $srcdir = $ComponentInfo{'mvapich2_gcc_hfi'}{'SrcDir'};
    return rpm_exists ("$srcdir", "user", "mvapich2_gcc_hfi");
}

# is component X presently installed on the system.  This is
# a quick check, not a "verify"
sub installed_mvapich2_gcc_hfi
{
    return  installed_generic_mpi("mvapich2", "gcc");
}

# what is the version installed on system.  Only
# called if installed_X is true.  versions are short strings displayed and
# logged, no operations are done (eg. only compare for equality)
sub installed_version_mvapich2_gcc_hfi
{
    return rpm_query_version_release_pkg ("mvapich2_gcc_hfi");
}

# only called if available_X.  Indicates version on
# media.  Will be compared with installed_version_X to determine if
# present installation is up to date.  Should return exact same format for
# version string so comparison of equality is possible.
sub media_version_mvapich2_gcc_hfi
{
    my $srcdir = $ComponentInfo{'mvapich2_gcc_hfi'}{'SrcDir'};
    my $rpm = rpm_resolve ("$srcdir", "user", "mvapich2_gcc_hfi");
    return rpm_query_version_release ($rpm);
}

# used to build/rebuild component on local system (if
# supported).  We support this for many items in comp_ofed.pl
# Other components (like SM) are
# not available in source and hence do not support this and simply
# implement a noop.
sub build_mvapich2_gcc_hfi
{
    my $osver = $_[0];
    my $debug = $_[1];
    my $build_temp = $_[2];
    my $force = $_[3];

    return 0;
}

# does this need to be reinstalled.  Mainly used for
# ofed due to subtle changes such as install prefix or kernel options
# which may force a reinstall.  You'll find this is a noop in most others.
sub need_reinstall_mvapich2_gcc_hfi
{
    my $install_list = shift ();
    my $installing_list = shift ();

    return "no";
}

# called for all components before they are installed.  Use to verify OS
# has proper dependent rpms installed.
sub check_os_prereqs_mvapich2_gcc_hfi
{
	return rpm_check_os_prereqs("mvapich2_gcc_hfi", "user");
}                              

# called for all components before they are installed.  Use
# to build things if needed, etc.
sub preinstall_mvapich2_gcc_hfi
{
    my $install_list = $_[0];
    my $installing_list = $_[1];

    my $full = "";
    my $rc;

    return 0;
}

# installs component.  also handles reinstall on top of
# existing installation and upgrade.
sub install_mvapich2_gcc_hfi
{
	install_generic_mpi("$_[0]", "$_[1]", "mvapich2", "gcc");
}

# called after all components are installed.
sub postinstall_mvapich2_gcc_hfi
{
    my $install_list = $_[0];     # total that will be installed when done
    my $installing_list = $_[1];  # what items are being installed/reinstalled
}

# uninstalls component.  May be called even if component is
# partially or not installed at all in which case should do its best to
# get rid or what might remain of component from a previously aborted
# uninstall or failed install
sub uninstall_mvapich2_gcc_hfi
{
    uninstall_generic_mpi("$_[0]", "$_[1]", "mvapich2", "gcc");
}

#############################################################################
##
##    MVAPICH2 Intel

# immediately stops or starts given component, only
# called if HasStart
sub start_mvapich2_intel_hfi
{
}

# is component X available on the install media (use of this
# allows for optional components in packaging or limited availability if a
# component isn't available on some OS/CPU combos)
sub stop_mvapich2_intel_hfi
{
}

# is component X available on the install media (use of this
# allows for optional components in packaging or limited availability if a
# component isn't available on some OS/CPU combos)
sub available_mvapich2_intel_hfi
{
    my $srcdir = $ComponentInfo{'mvapich2_intel_hfi'}{'SrcDir'};
    return rpm_exists ("$srcdir", "user", "mvapich2_intel_hfi");
}

# is component X presently installed on the system.  This is
# a quick check, not a "verify"
sub installed_mvapich2_intel_hfi
{
    return  installed_generic_mpi("mvapich2", "intel");
}

# what is the version installed on system.  Only
# called if installed_X is true.  versions are short strings displayed and
# logged, no operations are done (eg. only compare for equality)
sub installed_version_mvapich2_intel_hfi
{
    return rpm_query_version_release_pkg ("mvapich2_intel_hfi");
}

# only called if available_X.  Indicates version on
# media.  Will be compared with installed_version_X to determine if
# present installation is up to date.  Should return exact same format for
# version string so comparison of equality is possible.
sub media_version_mvapich2_intel_hfi
{
    my $srcdir = $ComponentInfo{'mvapich2_intel_hfi'}{'SrcDir'};
    my $rpm = rpm_resolve ("$srcdir", "user", "mvapich2_intel_hfi");
    return rpm_query_version_release ($rpm);
}

# used to build/rebuild component on local system (if
# supported).  We support this for many items in comp_ofed.pl
# Other components (like SM) are
# not available in source and hence do not support this and simply
# implement a noop.
sub build_mvapich2_intel_hfi
{
    my $osver = $_[0];
    my $debug = $_[1];
    my $build_temp = $_[2];
    my $force = $_[3];

    return 0;
}

# does this need to be reinstalled.  Mainly used for
# ofed due to subtle changes such as install prefix or kernel options
# which may force a reinstall.  You'll find this is a noop in most others.
sub need_reinstall_mvapich2_intel_hfi
{
    my $install_list = shift ();
    my $installing_list = shift ();

    return "no";
}

# called for all components before they are installed.  Use to verify OS
# has proper dependent rpms installed.
sub check_os_prereqs_mvapich2_intel_hfi
{
	# we allow this to install even if intel compiler runtime not available
	return rpm_check_os_prereqs("mvapich2_intel_hfi", "user");
}                              

# called for all components before they are installed.  Use
# to build things if needed, etc.
sub preinstall_mvapich2_intel_hfi
{
    my $install_list = $_[0];
    my $installing_list = $_[1];

    my $full = "";
    my $rc;

    return 0;
}

# installs component.  also handles reinstall on top of
# existing installation and upgrade.
sub install_mvapich2_intel_hfi
{
	install_generic_mpi("$_[0]", "$_[1]", "mvapich2", "intel");
}

# called after all components are installed.
sub postinstall_mvapich2_intel_hfi
{
    my $install_list = $_[0];     # total that will be installed when done
    my $installing_list = $_[1];  # what items are being installed/reinstalled
}

# uninstalls component.  May be called even if component is
# partially or not installed at all in which case should do its best to
# get rid or what might remain of component from a previously aborted
# uninstall or failed install
sub uninstall_mvapich2_intel_hfi
{
    uninstall_generic_mpi("$_[0]", "$_[1]", "mvapich2", "intel");
}

#############################################################################
##
##    MVAPICH2 PGI

# immediately stops or starts given component, only
# called if HasStart
sub start_mvapich2_pgi_hfi
{
}

# is component X available on the install media (use of this
# allows for optional components in packaging or limited availability if a
# component isn't available on some OS/CPU combos)
sub stop_mvapich2_pgi_hfi
{
}

# is component X available on the install media (use of this
# allows for optional components in packaging or limited availability if a
# component isn't available on some OS/CPU combos)
sub available_mvapich2_pgi_hfi
{
    my $srcdir = $ComponentInfo{'mvapich2_pgi_hfi'}{'SrcDir'};
    return rpm_exists ("$srcdir", "user", "mvapich2_pgi_hfi");
}

# is component X presently installed on the system.  This is
# a quick check, not a "verify"
sub installed_mvapich2_pgi_hfi
{
    return  installed_generic_mpi("mvapich2", "pgi");
}

# what is the version installed on system.  Only
# called if installed_X is true.  versions are short strings displayed and
# logged, no operations are done (eg. only compare for equality)
sub installed_version_mvapich2_pgi_hfi
{
    return rpm_query_version_release_pkg ("mvapich2_pgi_hfi");
}

# only called if available_X.  Indicates version on
# media.  Will be compared with installed_version_X to determine if
# present installation is up to date.  Should return exact same format for
# version string so comparison of equality is possible.
sub media_version_mvapich2_pgi_hfi
{
    my $srcdir = $ComponentInfo{'mvapich2_pgi_hfi'}{'SrcDir'};
    my $rpm = rpm_resolve ("$srcdir", "user", "mvapich2_pgi_hfi");
    return rpm_query_version_release ($rpm);
}

# used to build/rebuild component on local system (if
# supported).  We support this for many items in comp_ofed.pl
# Other components (like SM) are
# not available in source and hence do not support this and simply
# implement a noop.
sub build_mvapich2_pgi_hfi
{
    my $osver = $_[0];
    my $debug = $_[1];
    my $build_temp = $_[2];
    my $force = $_[3];

    return 0;
}

# does this need to be reinstalled.  Mainly used for
# ofed due to subtle changes such as install prefix or kernel options
# which may force a reinstall.  You'll find this is a noop in most others.
sub need_reinstall_mvapich2_pgi_hfi
{
    my $install_list = shift ();
    my $installing_list = shift ();

    return "no";
}

# called for all components before they are installed.  Use to verify OS
# has proper dependent rpms installed.
sub check_os_prereqs_mvapich2_pgi_hfi
{
	# we allow this to install even if pgi compiler runtime not available
	return rpm_check_os_prereqs("mvapich2_pgi_hfi", "user");
}                              

# called for all components before they are installed.  Use
# to build things if needed, etc.
sub preinstall_mvapich2_pgi_hfi
{
    my $install_list = $_[0];
    my $installing_list = $_[1];

    my $full = "";
    my $rc;

    return 0;
}

# installs component.  also handles reinstall on top of
# existing installation and upgrade.
sub install_mvapich2_pgi_hfi
{
	install_generic_mpi("$_[0]", "$_[1]", "mvapich2", "pgi");
}

# called after all components are installed.
sub postinstall_mvapich2_pgi_hfi
{
    my $install_list = $_[0];     # total that will be installed when done
    my $installing_list = $_[1];  # what items are being installed/reinstalled
}

# uninstalls component.  May be called even if component is
# partially or not installed at all in which case should do its best to
# get rid or what might remain of component from a previously aborted
# uninstall or failed install
sub uninstall_mvapich2_pgi_hfi
{
    uninstall_generic_mpi("$_[0]", "$_[1]", "mvapich2", "pgi");
}



#############################################################################
###
#+##    OpenMPI GCC CUDA
#
# immediately stops or starts given component, only
# called if HasStart
sub start_openmpi_gcc_cuda_hfi
{
}

# is component X available on the install media (use of this
# allows for optional components in packaging or limited availability if a
# component isn't available on some OS/CPU combos)
sub stop_openmpi_gcc_cuda_hfi
{
}

# is component X available on the install media (use of this
# allows for optional components in packaging or limited availability if a
# component isn't available on some OS/CPU combos)
sub available_openmpi_gcc_cuda_hfi
{
    my $srcdir = $ComponentInfo{'openmpi_gcc_cuda_hfi'}{'SrcDir'};
    return rpm_exists ("$srcdir", "user", "openmpi_gcc_cuda_hfi");
}

# is component X presently installed on the system.  This is
# a quick check, not a "verify"
sub installed_openmpi_gcc_cuda_hfi
{
    return  installed_generic_mpi("openmpi", "gcc_cuda");
}

# what is the version installed on system.  Only
# called if installed_X is true.  versions are short strings displayed and
# logged, no operations are done (eg. only compare for equality)
sub installed_version_openmpi_gcc_cuda_hfi
{
    return rpm_query_version_release_pkg ("openmpi_gcc_cuda_hfi");
}

# only called if available_X.  Indicates version on
# media.  Will be compared with installed_version_X to determine if
# present installation is up to date.  Should return exact same format for
# version string so comparison of equality is possible.
sub media_version_openmpi_gcc_cuda_hfi
{
    my $srcdir = $ComponentInfo{'openmpi_gcc_cuda_hfi'}{'SrcDir'};
    my $rpm = rpm_resolve ("$srcdir", "user", "openmpi_gcc_cuda_hfi");
    return rpm_query_version_release ($rpm);
}

# used to build/rebuild component on local system (if
# supported).  We support this for many items in comp_ofed.pl
# Other components (like SM) are
# not available in source and hence do not support this and simply
# implement a noop.
sub build_openmpi_gcc_cuda_hfi
{
    my $osver = $_[0];
    my $debug = $_[1];
    my $build_temp = $_[2];
    my $force = $_[3];

    return 0;
}

# does this need to be reinstalled.  Mainly used for
# ofed due to subtle changes such as install prefix or kernel options
# which may force a reinstall.  You'll find this is a noop in most others.
sub need_reinstall_openmpi_gcc_cuda_hfi
{
    my $install_list = shift ();
    my $installing_list = shift ();

    return "no";
}

# called for all components before they are installed.  Use to verify OS
# has proper dependent rpms installed.
sub check_os_prereqs_openmpi_gcc_cuda_hfi
{
       return rpm_check_os_prereqs("openmpi_gcc_cuda_hfi", "user");
}

# called for all components before they are installed.  Use
# to build things if needed, etc.
sub preinstall_openmpi_gcc_cuda_hfi
{
    my $install_list = $_[0];
    my $installing_list = $_[1];

    my $full = "";
    my $rc;

    return 0;
}

# installs component.  also handles reinstall on top of
# existing installation and upgrade.
sub install_openmpi_gcc_cuda_hfi
{
    install_generic_mpi("$_[0]", "$_[1]", "openmpi", "gcc_cuda");
}

# called after all components are installed.
sub postinstall_openmpi_gcc_cuda_hfi
{
    my $install_list = $_[0];     # total that will be installed when done
    my $installing_list = $_[1];  # what items are being installed/reinstalled
}

# uninstalls component.  May be called even if component is
# partially or not installed at all in which case should do its best to
# get rid or what might remain of component from a previously aborted
# uninstall or failed install
sub uninstall_openmpi_gcc_cuda_hfi
{
    uninstall_generic_mpi("$_[0]", "$_[1]", "openmpi", "gcc_cuda");
}

