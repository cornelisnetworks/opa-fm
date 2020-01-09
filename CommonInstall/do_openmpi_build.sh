#!/bin/bash
# BEGIN_ICS_COPYRIGHT8 ****************************************
# 
# Copyright (c) 2015-2018, Intel Corporation
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

# This file incorporates work covered by the following copyright and permission notice

# [ICS VERSION STRING: unknown]

# Copyright (c) 2006 Mellanox Technologies. All rights reserved.
#
# This Software is licensed under one of the following licenses:
#
# 1) under the terms of the "Common Public License 1.0" a copy of which is
#    available from the Open Source Initiative, see
#    http://www.opensource.org/licenses/cpl.php.
#
# 2) under the terms of the "The BSD License" a copy of which is
#    available from the Open Source Initiative, see
#    http://www.opensource.org/licenses/bsd-license.php.
#
# 3) under the terms of the "GNU General Public License (GPL) Version 2" a
#    copy of which is available from the Open Source Initiative, see
#    http://www.opensource.org/licenses/gpl-license.php.
#
# Licensee has the right to choose one of the above licenses.
#
# Redistributions of source code must retain the above copyright
# notice and one of the license notices.
#
# Redistributions in binary form must reproduce both the above copyright
# notice, one of the license notices in the documentation
# and/or other materials provided with the distribution.


# rebuild OpenMPI to target a specific compiler

ID=""
VERSION_ID=""

if [ -e /etc/os-release ]; then
	. /etc/os-release
else
    echo /etc/os-release is not available !!!
fi

if [[ ( "$ID" == "rhel"  &&  $(echo "$VERSION_ID > 7.3" | bc -l) == 1 ) || \
	( "$ID" == "sles"  && $(echo "$VERSION_ID > 12.2" | bc -l) == 1 ) ]]; then
    PREREQ=("rdma-core-devel" "mpi-selector")
else
    PREREQ=("libibverbs-devel" "librdmacm-devel" "mpi-selector")
fi

CheckPreReqs()
{
	e=0;
	i=0;
	while [ $i -lt ${#PREREQ[@]} ]; do
		rpm -qa | grep ${PREREQ[$i]} >/dev/null
		if [ $? -ne 0 ]; then
			if [ $e -eq 0 ]; then
				 echo
			fi
			echo "ERROR: Before re-compiling OpenMPI you must first install the ${PREREQ[$i]} package." >&2
			e+=1;
		fi
		i=$((i+1))
	done

	if [ $e -ne 0 ]; then
		if [ $e -eq 1 ]; then
			echo "ERROR: Cannot build. Please install the missing package before re-trying." >&2
		else
			echo "ERROR: Cannot build. Please install the listed packages before re-trying." >&2
		fi
		echo
		exit 2
	fi
}

Usage()
{
	echo "Usage: do_openmpi_build [-d] [-Q|-O] [config_opt [install_dir]]" >&2
	echo "       -d - use default settings for openmpi options" >&2
	echo "            if omitted, will be prompted for each option" >&2
	echo "       -Q - build the MPI targeted for the PSM API." >&2
	echo "       -O - build the MPI targeted for the Omni-path HFI PSM2 and OFI API." >&2
	echo "       -C - build the MPI targeted for the Omnipath HFI PSM with CUDA." >&2
	echo "       config_opt - a compiler selection option (gcc, pathscale, pgi or intel)" >&2
	echo "             if config_opt is not specified, the user will be prompted" >&2
	echo "             based on compilers found on this system" >&2
	echo "       install_dir - where to install MPI, see MPICH_PREFIX below" >&2
	echo "" >&2
	echo "Environment:" >&2
	echo "    STACK_PREFIX - where to find IB stack." >&2
	echo "    BUILD_DIR - temporary directory to use during build of MPI" >&2
	echo "            Default is /var/tmp/Intel-openmpi" >&2
	echo "    MPICH_PREFIX - selects location for installed MPI" >&2
	echo "            default is /usr/mpi/<COMPILER>/openmpi-<VERSION>" >&2
	echo "            where COMPILER is selected compiler (gcc, pathscale, etc above)" >&2
	echo "            VERSION is openmpi version (eg. 1.0.0)" >&2
	echo "    CONFIG_OPTIONS - additional OpenMPI configuration options to be" >&2
	echo "            specified via configure_options parameter to srpm" >&2
	echo "            Default is ''" >&2
	echo "    INSTALL_ROOT - location of system image in which to install." >&2
	echo "            Default is '/'" >&2
	echo "" >&2
	echo "The RPMs built during this process will be installed on this system" >&2
	echo "they can also be found in /usr/src/opa/MPI" >&2
	exit 2
}

unset MAKEFLAGS

ARCH=$(uname -m | sed -e s/ppc/PPC/ -e s/powerpc/PPC/ -e s/i.86/IA32/ -e s/ia64/IA64/ -e s/x86_64/X86_64/)

target_cpu=$(rpm --eval '%{_target_cpu}')
dist_rpm_rel_int=0
if [ "$ARCH" = "PPC64" -a -f /etc/issue -a -f /etc/SuSE-release ]
then
	# needed to test for SLES 10 SP1 on PPC64 below
	dist_rpm_rel=$(rpm --queryformat "[%{RELEASE}]\n" -q $(rpm -qf /etc/issue)|uniq)
	dist_rpm_rel_major="$(echo $dist_rpm_rel|cut -f1 -d.)"
	dist_rpm_rel_minor="$(echo $dist_rpm_rel|cut -f2 -d.)"
	# convert version to a 4 digit integer
	if [ $dist_rpm_rel_major -lt 10 ]
	then
		dist_rpm_rel_major="0$dist_rpm_rel_major";
	fi
	if [ $dist_rpm_rel_minor -lt 10 ]
	then
		dist_rpm_rel_minor="0$dist_rpm_rel_minor";
	fi
	dist_rpm_rel_int="$dist_rpm_rel_major$dist_rpm_rel_minor"
fi

nocomp()
{
	echo "ERROR: No Compiler Found, unable to Rebuild OpenMPI MPI" >&2
	exit 1
}

# determine if the given tool/compiler exists in the PATH
have_comp()
{
	type $1 > /dev/null 2>&1
	return $?
}

# global $ans set to 1 for yes or 0 for no
get_yes_no()
{
	local prompt default input
	prompt="$1"
	default="$2"
	while true
	do
		echo -n "$prompt [$default]:"
		read input
		if [ "x$input" = x ]
		then
			input="$default"
		fi
		case "$input" in
		[Yy]*)	ans=1; break;;
		[Nn]*)	ans=0; break;;
		esac
	done
}

skip_prompt=n
iflag=n	# undocumented option, build in context of install
Qflag=n
Oflag=n
Cflag=n
Vflag=n # undocumented option, build verbs only transport
while getopts "idQOCV" o
do
	case "$o" in
	i) iflag=y;;
	Q) Qflag=y;;
	O) Oflag=y;;
	C) Cflag=y;;
	V) Vflag=y;;
	d) skip_prompt=y;;
	*) Usage;;
	esac
done
shift $((OPTIND -1))
if [ $# -gt 2 ]
then
	Usage
fi

if [[  "$Vflag" == "y"  &&  \
	( "$Qflag" == "y" || "$Oflag" == "y" || "$Cflag" == "y"  ) ]]; then
	echo "ERROR: Option -V cannot be used with any other" >&2
	exit 1
fi

if [ "$(/usr/bin/id -u)" != 0 ]
then
	echo "ERROR: You must be 'root' to run this program" >&2
	exit 1
fi
if [ "$iflag" = n ]
then
	cd /usr/src/opa/MPI
	if [ $? != 0 ]
	then
		echo "ERROR: Unable to cd to /usr/src/opa/MPI" >&2
		exit 1
	fi
fi

echo
echo "IFS OpenMPI MPI Library/Tools rebuild"

if [ x"$1" != x"" ]
then
	compiler="$1"
else
	compiler=none
	choices=""
	# open MPI does not require fortran compiler, so just check for C
	if have_comp gcc
	then
		choices="$choices gcc"
	fi
	if have_comp pathcc
	then
		choices="$choices pathscale"
	fi
	if have_comp pgcc
	then
		choices="$choices pgi"
	fi
	if have_comp icc
	then
		choices="$choices intel"
	fi
	if [ x"$choices" = x ]
	then
		nocomp
	else
		PS3="Select Compiler: "
		select compiler in $choices
		do
			case "$compiler" in
			gcc|pathscale|pgi|intel) break;;
			esac
		done
	fi
fi

case "$compiler" in
gcc|pathscale|pgi|intel) >/dev/null;;
*)
	echo "ERROR: Invalid Compiler selection: $compiler" >&2
	exit 1;;
esac
shift
if [ ! -z "$1" ]
then
	export MPICH_PREFIX="$1"
fi

# now get openmpi options.
if [ "$Vflag" != y ]
then
	if [ "$skip_prompt" != y -a "$Qflag" != y ]
	then
		if rpm -qa|grep infinipath-psm-devel >/dev/null 2>&1
		then
			echo
			get_yes_no "Build for True Scale HCA PSM" "y"
			if [ "$ans" = 1 ]
			then
				Qflag=y
			fi
		fi
	fi

	if [ "$skip_prompt" != y -a "$Oflag" != y ]
	then
		if  rpm -qa|grep libpsm2-devel >/dev/null 2>&1  &&
		    rpm -qa|grep libfabric-devel >/dev/null 2>&1
		then
			echo
			get_yes_no "Build for Omnipath HFI PSM2 and OFI" "y"
			if [ "$ans" = 1 ]
			then
				Oflag=y
			fi
		fi
	fi

	if [ "$skip_prompt" != y -a "$Cflag" != y ]
	then
		if  rpm -qa|grep libpsm2-devel >/dev/null 2>&1  &&
		    rpm -qa|grep cuda-cudart-dev >/dev/null 2>&1
		then
			echo
			get_yes_no "Build for Omnipath HFI PSM2 with Cuda" "y"
			if [ "$ans" = 1 ]
			then
				Cflag=y
			fi
		fi
    fi
fi
# if -d (skip_prompt) the only option provided, ./configure will run with
# no paramters and build what is auto-detected

openmpi_conf_psm=''

if [ "$Vflag" = y ]
then
	# The openmpi configure script complains about enable-mca-no-build
	# not being a supported option, but then actually executes it correctly.
	#openmpi_conf_psm='--enable-mca-no-build=mtl-psm'
	openmpi_conf_psm='--with-psm=no --with-psm2=no --with-libfabric=no --enable-mca-no-build=mtl-psm '
	openmpi_path_suffix=
	openmpi_rpm_suffix=
	interface=verbs
else
	if [ "$Qflag" = y ]
	then
		PREREQ+=('infinipath-psm-devel')
		openmpi_conf_psm='--with-psm=/usr '
		# PSM indicated by qlc suffix so user can ID PSM vs verbs or PSM2 MPIs
		openmpi_path_suffix="-qlc"
		openmpi_rpm_suffix="_qlc"
		interface=psm
	fi

	if [ "$Oflag" = y ]
	then
		PREREQ+=('libpsm2-devel' 'libfabric-devel')
		openmpi_conf_psm=" $openmpi_conf_psm --with-psm2=/usr --with-libfabric=/usr "
		# PSM2 indicated by hfi suffix so user can ID from PSM or verbs MPIs
		openmpi_path_suffix="-hfi"
		openmpi_rpm_suffix="_hfi"
		interface=psm
	fi

	if [ "$Cflag" = y ]
	then
		PREREQ+=('libpsm2-devel' 'cuda-cudart-dev')
		openmpi_conf_psm=" $openmpi_conf_psm --with-psm2=/usr --with-cuda=/usr/local/cuda "
		# CUDA indicated by -cuda suffix so user can ID  from PSM2 without cuda, PSM or verbs MPIs
		openmpi_path_suffix="-cuda-hfi"
		openmpi_rpm_suffix="_cuda_hfi"
		interface=psm
	fi
fi

CheckPreReqs

if [ "$ARCH" = "PPC64" -a \
	\( ! -f /etc/SuSE-release -o "$dist_rpm_rel_int" -le "1502" \) ]	# eg. 15.2
then
	export LDFLAGS="-m64 -g -O2 -L/usr/lib64 -L/usr/X11R6/lib64"
	export CFLAGS="-m64 -g -O2"
	export CPPFLAGS="-m64 -g -O2"
	export CXXFLAGS="-m64 -g -O2"
	export FFLAGS="-m64 -g -O2"
	export F90FLAGS="-m64 -g -O2"
	export LDLIBS="-m64 -g -O2 -L/usr/lib64 -L/usr/X11R6/lib64"
else
	# just to be safe
	unset LDFLAGS
	unset CFLAGS
	unset CPPFLAGS
	unset CXXFLAGS
	unset FFLAGS
	unset F90FLAGS
	unset LDLIBS
fi

logfile=make.openmpi.$interface.$compiler
(
	STACK_PREFIX=${STACK_PREFIX:-/usr}
	BUILD_DIR=${BUILD_DIR:-/var/tmp/Intel-openmpi}
	BUILD_ROOT="$BUILD_DIR/build";
	RPM_DIR="$BUILD_DIR/OFEDRPMS";
	DESTDIR=/usr/src/opa/MPI
	if [ "$iflag" = n ]
	then
		openmpi_srpm=/usr/src/opa/MPI/openmpi-*.src.rpm
		mpitests_srpm=/usr/src/opa/MPI/mpitests-*.src.rpm
	else
		openmpi_srpm=./SRPMS/openmpi-*.src.rpm
		mpitests_srpm=./SRPMS/mpitests-*.src.rpm
	fi
	openmpi_version=$(ls $openmpi_srpm 2>/dev/null|head -1|cut -f2 -d-)

	# For RHEL7x: %{?dist} resolves to '.el7'. For SLES, an empty string
	# E.g. on rhel7.x: openmpi_gcc_hfi-2.1.2-11.el7.x86_64.rpm; on SLES openmpi_gcc_hfi-2.1.2-11.x86_64.rpm
	openmpi_fullversion=$(ls $openmpi_srpm 2>/dev/null|head -1|cut -f2- -d-|sed -e 's/.src.rpm//')$(rpm --eval %{?dist})
	mpitests_version=$(ls $mpitests_srpm 2>/dev/null|head -1|cut -f2 -d-)
	mpitests_fullversion=$(ls $mpitests_srpm 2>/dev/null|head -1|cut -f2- -d-|sed -e 's/.src.rpm//')
	MPICH_PREFIX=${MPICH_PREFIX:-$STACK_PREFIX/mpi/$compiler/openmpi-$openmpi_version$openmpi_path_suffix}
	CONFIG_OPTIONS=${CONFIG_OPTIONS:-""}

	if [ x"$openmpi_version" = x"" ]
	then
		echo "Error $openmpi_srpm: Not Found"
		exit 1
	fi
	if [ x"$mpitests_version" = x"" ]
	then
		echo "Error $mpitests_srpm: Not Found"
		exit 1
	fi

	echo "Environment:"
	env
	echo "=========================================================="
	echo
	echo "Build Settings:"
	echo "STACK_PREFIX='$STACK_PREFIX'"
	echo "BUILD_DIR='$BUILD_DIR'"
	echo "MPICH_PREFIX='$MPICH_PREFIX'"
	echo "CONFIG_OPTIONS='$CONFIG_OPTIONS'"
	echo "OpenMPI Version: $openmpi_version"
	echo "OpenMPI Full Version: $openmpi_fullversion"
	echo "mpitests Version: $mpitests_version"
	echo "mpitests Full Version: $mpitests_fullversion"
	echo "=========================================================="
	if [ "$iflag" = n ]
	then
		echo "MPICH_PREFIX='$MPICH_PREFIX'"> /usr/src/opa/MPI/.mpiinfo
		#echo "MPI_RUNTIME='$MPICH_PREFIX/bin $MPICH_PREFIX/lib* $MPICH_PREFIX/etc $MPICH_PREFIX/share $MPICH_PREFIX/tests'">> /usr/src/opa/MPI/.mpiinfo
		echo "MPI_RPMS='openmpi_$compiler$openmpi_rpm_suffix-$openmpi_fullversion.$target_cpu.rpm mpitests_openmpi_$compiler$openmpi_rpm_suffix-$mpitests_fullversion.$target_cpu.rpm'">> /usr/src/opa/MPI/.mpiinfo
		chmod +x /usr/src/opa/MPI/.mpiinfo
	fi

	echo
	echo "Cleaning build tree..."
	rm -rf $BUILD_DIR > /dev/null 2>&1

	echo "=========================================================="
	echo "Building OpenMPI MPI $openmpi_version Library/Tools..."
	mkdir -p $BUILD_ROOT $RPM_DIR/BUILD $RPM_DIR/RPMS $RPM_DIR/SOURCES $RPM_DIR/SPECS $RPM_DIR/SRPMS

	if [ "$ARCH" = "PPC64" -o "$ARCH" = "X86_64" ]
	then
		openmpi_lib="lib64"
	else
		openmpi_lib="lib"
	fi

	use_default_rpm_opt_flags=1
	disable_auto_requires=""
	openmpi_ldflags=""
	openmpi_wrapper_cxx_flags=""

	# need to create proper openmpi_comp_env value for OpenMPI builds
	if [ "$ARCH" = "PPC64" -a \
		\( -f /etc/SuSE-release -a "$dist_rpm_rel_int" -gt "1502" \) ]	# eg. 15.2
	then
		openmpi_comp_env='LDFLAGS="-m64 -O2 -L/usr/lib/gcc/powerpc64-suse-linux/4.1.2/64"'
	else
		openmpi_comp_env=""
	fi

	case "$compiler" in
	gcc)
		if [[ ( "$ID" == "rhel"  &&  $(echo "$VERSION_ID >= 8.0" | bc -l) == 1 ) ]]; then
			openmpi_comp_env="$openmpi_comp_env CC=gcc CFLAGS="-O3 -fPIC""
		else
			openmpi_comp_env="$openmpi_comp_env CC=gcc CFLAGS=-O3"
		fi
		if have_comp g++
		then
			openmpi_comp_env="$openmpi_comp_env CXX=g++"
		else
			openmpi_comp_env="$openmpi_comp_env --disable-mpi-cxx"
		fi
		if have_comp gfortran
		then
			openmpi_comp_env="$openmpi_comp_env F77=gfortran FC=gfortran"
		elif have_comp g77
		then
			openmpi_comp_env="$openmpi_comp_env F77=g77 --disable-mpi-f90"
		else
			openmpi_comp_env="$openmpi_comp_env --disable-mpi-f77 --disable-mpi-f90"
		fi;;

	pathscale)
		openmpi_comp_env="$openmpi_comp_env CC=pathcc"
		disable_auto_requires="--define 'disable_auto_requires 1'"
		if have_comp pathCC
		then
			openmpi_comp_env="$openmpi_comp_env CXX=pathCC"
		else
			openmpi_comp_env="$openmpi_comp_env --disable-mpi-cxx"
		fi
		if have_comp pathf90
		then
			openmpi_comp_env="$openmpi_comp_env F77=pathf90 FC=pathf90"
		else
			openmpi_comp_env="$openmpi_comp_env --disable-mpi-f77 --disable-mpi-f90"
		fi
		# test for fedora core 6 or redhat EL5
		if { [ -f /etc/fedora-release ] && { uname -r|grep fc6; }; } \
			|| { [ -f /etc/redhat-release ] && { uname -r|grep el5; }; }
		then
			use_default_rpm_opt_flags=0
		fi;;

	pgi)
		disable_auto_requires="--define 'disable_auto_requires 1'"
		openmpi_comp_env="$openmpi_comp_env CC=pgcc"
		use_default_rpm_opt_flags=0
		if have_comp pgCC
		then
			openmpi_comp_env="$openmpi_comp_env CXX=pgCC"
			# See http://www.pgroup.com/userforum/viewtopic.php?p=2371
			openmpi_wrapper_cxx_flags="-fpic"
		else
			openmpi_comp_env="$openmpi_comp_env --disable-mpi-cxx"
		fi
		if have_comp pgf77
		then
			openmpi_comp_env="$openmpi_comp_env F77=pgf77"
		else
			openmpi_comp_env="$openmpi_comp_env --disable-mpi-f77"
		fi
		if have_comp pgf90
		then
			openmpi_comp_env="$openmpi_comp_env FC=pgf90 FCFLAGS=-O2"
		else
			openmpi_comp_env="$openmpi_comp_env --disable-mpi-f90"
		fi;;

	intel)
		disable_auto_requires="--define 'disable_auto_requires 1'"
		openmpi_comp_env="$openmpi_comp_env CC=icc"
		if have_comp icpc
		then
			openmpi_comp_env="$openmpi_comp_env CXX=icpc"
		else
			openmpi_comp_env="$openmpi_comp_env --disable-mpi-cxx"
		fi
		if have_comp ifort
		then
			openmpi_comp_env="$openmpi_comp_env F77=ifort FC=ifort"
		else
			openmpi_comp_env="$openmpi_comp_env --disable-mpi-f77 --disable-mpi-f90"
		fi;;

	*)
		echo "ERROR: Invalid compiler"
		exit 1;;
	esac

	if [ "$ARCH" = "PPC64" ]
	then
		openmpi_comp_env="$openmpi_comp_env --disable-mpi-f77 --disable-mpi-f90"
		# In the ppc64 case, add -m64 to all the relevant
		# flags because it's not the default.  Also
		# unconditionally add $OMPI_RPATH because even if
		# it's blank, it's ok because there are other
		# options added into the ldflags so the overall
		# string won't be blank.
		openmpi_comp_env="$openmpi_comp_env CFLAGS=\"-m64 -O2\" CXXFLAGS=\"-m64 -O2\" FCFLAGS=\"-m64 -O2\" FFLAGS=\"-m64 -O2\""
		openmpi_comp_env="$openmpi_comp_env --with-wrapper-ldflags=\"-g -O2 -m64 -L/usr/lib64\" --with-wrapper-cflags=-m64"
		openmpi_comp_env="$openmpi_comp_env --with-wrapper-cxxflags=-m64 --with-wrapper-fflags=-m64 --with-wrapper-fcflags=-m64"
		openmpi_wrapper_cxx_flags="$openmpi_wrapper_cxx_flags -m64";
	fi

	openmpi_comp_env="$openmpi_comp_env --enable-mpirun-prefix-by-default"
	if [ x"$openmpi_wrapper_cxx_flags" != x ]
	then
		openmpi_comp_env="$openmpi_comp_env --with-wrapper-cxxflags=\"$openmpi_wrapper_cxx_flags\""
	fi

	pref_env=
	if [ "$STACK_PREFIX" != "/usr" ]
	then
		pref_env="$pref_env LD_LIBRARY_PATH=$STACK_PREFIX/lib64:$STACK_PREFIX/lib:\$LD_LIBRARY_PATH"
	fi

	if [ "$Cflag" = y ]
	then
		pref_env="$pref_env MPI_STRESS_CUDA=1"
	else
		# HWLOC component auto detects CUDA and will use it even if it is NOT
		# a CUDA OMPI build. So, tell HWLOC to ignore CUDA (if found on the system)
		# when not creating a CUDA build.
		pref_env="$pref_env enable_gl=no"
	fi

	cmd="$pref_env rpmbuild --rebuild \
				--define '_topdir $RPM_DIR' \
				--buildroot '$BUILD_ROOT' \
				--define 'build_root $BUILD_ROOT' \
				--target $target_cpu \
				--define '_name openmpi_$compiler$openmpi_rpm_suffix' \
				--define 'compiler $compiler' \
				--define 'mpi_selector $STACK_PREFIX/bin/mpi-selector' \
				--define 'use_mpi_selector 1' \
				--define 'install_shell_scripts 1' \
				--define 'shell_scripts_basename mpivars' \
				--define '_usr $STACK_PREFIX' \
				--define 'ofed 0' \
				--define '_prefix $MPICH_PREFIX' \
				--define '_defaultdocdir $MPICH_PREFIX/doc/..' \
				--define '_mandir %{_prefix}/share/man' \
				--define 'mflags -j 4' \
				--define 'configure_options $CONFIG_OPTIONS $openmpi_ldflags --with-verbs=$STACK_PREFIX --with-verbs-libdir=$STACK_PREFIX/$openmpi_lib $openmpi_comp_env $openmpi_conf_psm --with-devel-headers --disable-oshmem' \
				--define 'use_default_rpm_opt_flags $use_default_rpm_opt_flags' \
				$disable_auto_requires"
	cmd="$cmd \
				$openmpi_srpm"
	echo "Executing: $cmd"
	eval $cmd
	if [ $? != 0 ]
	then
		echo "error: openmpi_$compiler$openmpi_rpm_suffix Build ERROR: bad exit code"
		exit 1
	fi
	if [ "$iflag" = n ]
	then
		cp $RPM_DIR/RPMS/$target_cpu/openmpi_$compiler$openmpi_rpm_suffix-$openmpi_fullversion.$target_cpu.rpm $DESTDIR
	fi

	echo "=========================================================="
	echo "Installing OpenMPI MPI $openmpi_version Library/Tools..."
	rpmfile=$RPM_DIR/RPMS/$target_cpu/openmpi_$compiler$openmpi_rpm_suffix-$openmpi_fullversion.$target_cpu.rpm

	# need force for reinstall case
	if [ x"$INSTALL_ROOT" != x"" -a x"$INSTALL_ROOT" != x"/" ]
	then
		tempfile=/var/tmp/rpminstall.tmp.rpm
		mkdir -p $INSTALL_ROOT/var/tmp
		cp $rpmfile $INSTALL_ROOT$tempfile
		#chroot /$INSTALL_ROOT rpm -ev --nodeps openmpi_$compiler$openmpi_rpm_suffix-$openmpi_version
		chroot /$INSTALL_ROOT rpm -Uv --force $tempfile
		rm -f $tempfile
	else
		#rpm -ev --nodeps openmpi_$compiler$openmpi_rpm_suffix-$openmpi_version
		rpm -Uv --force $rpmfile
	fi

	echo "=========================================================="
	echo "Building test programs $mpitests_version for OpenMPI MPI $openmpi_version..."
	# mpitests uses buildroot instead of build_root, play it safe for future
	# and define both
	cmd="$pref_env rpmbuild --rebuild \
				--define '_topdir $RPM_DIR' \
				--buildroot '$BUILD_ROOT' \
				--define 'buildroot $BUILD_ROOT' \
				--define 'build_root $BUILD_ROOT' \
				--target $target_cpu \
				--define '_name mpitests_openmpi_$compiler$openmpi_rpm_suffix' \
				--define 'root_path /' \
				--define '_usr $STACK_PREFIX' \
				--define 'path_to_mpihome $MPICH_PREFIX' \
				--define 'enable_cuda $Cflag' \
				$disable_auto_requires \
				$mpitests_srpm"
	echo "Executing: $cmd"
	eval $cmd
	if [ $? != 0 ]
	then
		echo "error: mpitests_openmpi_$compiler$openmpi_rpm_suffix Build ERROR: bad exit code"
		exit 1
	fi

	if [ "$iflag" = n ]
	then
		mv $RPM_DIR/RPMS/$target_cpu/mpitests_openmpi_$compiler$openmpi_rpm_suffix-$mpitests_fullversion.$target_cpu.rpm $DESTDIR

		echo "=========================================================="
		echo "Installing test programs $mpitests_version for OpenMPI MPI $openmpi_version..."
		rpmfile=$DESTDIR/mpitests_openmpi_$compiler$openmpi_rpm_suffix-$mpitests_fullversion.$target_cpu.rpm
		# need force for reinstall case
		if [ x"$INSTALL_ROOT" != x"" -a x"$INSTALL_ROOT" != x"/" ]
		then
			tempfile=/var/tmp/rpminstall.tmp.rpm
			mkdir -p $INSTALL_ROOT/var/tmp
			cp $rpmfile $INSTALL_ROOT$tempfile
			chroot /$INSTALL_ROOT rpm -Uv --force $tempfile
			rm -f $tempfile
		else
			rpm -Uv --force $rpmfile
		fi

		rm -rf $BUILD_DIR
	fi
) 2>&1|tee $logfile.res
set +x

# review log for errors and warnings
# disable output of build warnings, way too many
#echo
#echo "Build Warnings:"
# ignore the warning for old C++ header usage in sample programs
egrep 'warning:' $logfile.res |sort -u |
	egrep -v 'at least one deprecated or antiquated header.*C\+\+ includes' > $logfile.warn
#cat $logfile.warn
#echo

#egrep 'error:|Error | Stop' $logfile.res| sort -u |
#	egrep -v 'error: this file was generated for autoconf 2.61.' > $logfile.err
egrep 'error:|Error | Stop' $logfile.res| sort -u |
	egrep -v 'configure: error: no BPatch.h found; check path for Dyninst package|configure: error: no vtf3.h found; check path for VTF3 package|configure: error: MPI Correctness Checking support cannot be built inside Open MPI|configure: error: no bmi.h found; check path for BMI package first...|configure: error: no ctool/ctool.h found; check path for CTool package first...|configure: error: no cuda.h found; check path for CUDA Toolkit first...|configure: error: no cuda_runtime_api.h found; check path for CUDA Toolkit first...|configure: error: no cupti.h found; check path for CUPTI package first...|configure: error: no f2c.h found; check path for CLAPACK package first...|configure: error: no jvmti.h found; check path for JVMTI package first...|configure: error: no libcpc.h found; check path for CPC package first...|configure: error: no tau_instrumentor found; check path for PDToolkit first...|configure: error: no unimci-config found; check path for UniMCI package first...|"Error code:|"Unknown error:|strerror_r|configure: error: CUPTI API version could not be determined...|asprintf(&msg, "Unexpected sendto() error: errno=%d (%s)",' > $logfile.err

if [ -s $logfile.err ]
then
	echo "Build Errors:"
	sort -u $logfile.err
	echo
	echo "FAILED Build, errors detected"
	exit 1
elif [ -s $logfile.warn ]
then
	# at present lots of warnings are expected
	echo "SUCCESSFUL Build, no errors detected"
	exit 0

	# No warnings are expected
	echo "QUESTIONABLE Build, warnings detected"
	exit 1
else
	echo "SUCCESSFUL Build, no errors detected"
	exit 0
fi
