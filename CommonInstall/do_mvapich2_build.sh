#!/bin/bash
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

# This file incorporates work covered by the following copyright and permission notice 

#[ICS VERSION STRING: unknown]

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


# rebuild MVAPICH2 to target a specific compiler

PREREQ=("libibverbs-devel" "librdmacm-devel" "mpi-selector")

CheckPreReqs()
{
	e=0;
	i=0;
	while [ $i -lt ${#PREREQ[@]} ]; do
		rpm -q ${PREREQ[$i]} >/dev/null
		if [ $? -ne 0 ]; then
			if [ $e -eq 0 ]; then
				 echo
			fi
			echo "ERROR: Before re-compiling mvapich2 you must first install the ${PREREQ[$i]} package." >&2
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
	echo "Usage: do_mvapich2_build [-d] [-Q|-O] [config_opt [install_dir]]" >&2
	echo "       -d - use default settings for various MVAPICH2 capabilities" >&2
	echo "            if omitted, will be prompted for each capability" >&2
	echo "       -Q - build the MPI targeted for the PSM API." >&2
	echo "       -O - build the MPI targeted for the PSM2 API." >&2
	echo "       config_opt - a compiler selection option (gcc, pathscale, pgi or intel)" >&2
	echo "            if config_opt is not specified, the user will be prompted" >&2
	echo "            based on compilers found on this system" >&2
	echo "       install_dir - where to install MPI, see MPICH_PREFIX below" >&2
	echo "" >&2
	echo "Environment:" >&2
	echo "    STACK_PREFIX - where to find IB stack." >&2
	echo "    BUILD_DIR - temporary directory to use during build of MPI" >&2
	echo "            Default is /var/tmp/Intel-mvapich2" >&2
	echo "    MPICH_PREFIX - selects location for installed MPI" >&2
	echo "            default is /usr/mpi/<COMPILER>/mvapich2-<VERSION>" >&2
	echo "            where COMPILER is selected compiler (gcc, pathscale, etc above)" >&2
	echo "            VERSION is mvapich2 version (eg. 1.0.0)" >&2
	echo "    CONFIG_OPTIONS - additional MVAPICH2 configuration options to be" >&2
	echo "            specified to srpm" >&2
	echo "            Default is ''" >&2
	echo "    INSTALL_ROOT - location of system image in which to install." >&2
	echo "            Default is '/'" >&2
	echo "" >&2
	echo "The RPMs built during this process will be installed on this system" >&2
	echo "they can also be found in /usr/src/opa/MPI" >&2
	exit 2
}

unset MAKEFLAGS

# fixup possible missing path to X11
export PATH=$PATH:/usr/X11R6/bin

ARCH=$(uname -m | sed -e s/ppc/PPC/ -e s/powerpc/PPC/ -e s/i.86/IA32/ -e s/ia64/IA64/ -e s/x86_64/X86_64/)

target_cpu=$(rpm --eval '%{_target_cpu}')
dist_rpm_rel_int=0
if [ "$ARCH" = "PPC64" -a  -f /etc/issue ]
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

nofort()
{
	echo "ERROR: No Fortran Compiler Found, unable to Rebuild MVAPICH2 MPI" >&2
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
while getopts "idQO" o
do
	case "$o" in
	i) iflag=y;;
	Q) Qflag=y;;
	O) Oflag=y;;
	d) skip_prompt=y;;
	*) Usage;;
	esac
done
shift $((OPTIND -1))
if [ $# -gt 2 ]
then
	Usage
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
echo "OFA MVAPICH2 MPI Library/Tools rebuild"

if [ x"$1" != x"" ]
then
	compiler="$1"
else
	compiler=none
	choices=""
	if have_comp gcc && { have_comp g77 || have_comp gfortran; }
	then
		choices="$choices gcc"
	fi
	if have_comp pathcc && have_comp pathcCC && have_comp pathf90
	then
		choices="$choices pathscale"
	fi
	if have_comp pgcc && have_comp pgf77 && have_comp pgf90
	then
		choices="$choices pgi"
	fi
	if have_comp icc && have_comp icpc && have_comp ifort
	then
		choices="$choices intel"
	fi
	if [ x"$choices" = x ]
	then
		nofort
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

PS3="Select MVAPICH2 Implementation (ofa recommended): "
choices=("ofa")
mvapich2_conf_impl=ofa
mvapich2_conf_impl_define='impl ofa'
interface=verbs

# Now get MVAPICH2 capability options. Note that you can't build for
# TrueScale and Omnipath at the same time.
if [ "$skip_prompt" == "n" -a "$Qflag" == "n" -a "$Oflag" == "n" ]
then
	echo
	# only have a choice if psm is installed
	if rpm -qa|grep infinipath-devel >/dev/null 2>&1
	then
		choices+=("ts-psm")
		PS3="Select MVAPICH2 Implementation (ts-psm recommended): "
	fi
	if rpm -qa|grep libpsm2 >/dev/null 2>&1
	then
		choices+=("opa-psm")
		PS3="Select MVAPICH2 Implementation (opa-psm recommended): "
	fi
	if [ ${#choices[@]} -gt 0 ] 
	then
		select mvapich2_conf_impl in ${choices[*]}
		do
			case "$mvapich2_conf_impl" in
			ofa)
				interface=verbs
				break;;
			ts-psm)
				interface=psm
		   		Qflag=y
				break;;
			opa-psm)
				interface=psm
		   		Oflag=y
				break;;
			esac
		done
	else
		echo "ERROR: No RDMA stack available."
		exit 1
	fi
elif [ "$Qflag" == "y" -o "$Oflag" == "y" ]
then
	interface=psm
else 
	interface=verbs
fi

case $interface in
	verbs)
		mvapich2_conf_impl_define="impl $mvapich2_conf_impl"
		mvapich2_path_suffix=
		mvapich2_rpm_suffix=
		;;
	psm)
		mvapich2_conf_impl=psm
#		mvapich2_conf_impl_define="channel ch3:psm"
		mvapich2_conf_impl_define="impl psm2"
		mvapich2_conf_psm=
		if [ "$Oflag" == "y" ]
		then
			# PSM indicated by qlc suffix so user can ID PSM vs verbs MPIs
			mvapich2_path_suffix="-hfi"
			mvapich2_rpm_suffix="_hfi"
   			PREREQ+=("libpsm2")
		else
			# PSM indicated by qlc suffix so user can ID PSM vs verbs MPIs
			mvapich2_path_suffix="-qlc"
			mvapich2_rpm_suffix="_qlc"
   			PREREQ+=("infinipath-devel")
		fi
		;;
esac

CheckPreReqs

if [ "$skip_prompt" = y ]
then
	mvapich2_conf_romio=1
	mvapich2_conf_shared_libs=1
	mvapich2_conf_ckpt=0
	mvapich2_conf_blcr_home=""
else
	get_yes_no "Enable ROMIO support" "y"
	mvapich2_conf_romio=$ans

	get_yes_no "Enable shared library support" "y"
	mvapich2_conf_shared_libs=$ans

	if [ "$mvapich2_conf_impl" = "ofa" ]
	then
		while true
		do
			get_yes_no "Enable Checkpoint-Restart support" "n"
			mvapich2_conf_ckpt=$ans
			if [ $ans = 1 ]
			then
				echo -n "BLCR installation directory [or NONE if not installed]:"
				read mvapich2_conf_blcr_home
				if [ "$mvapich2_conf_blcr_home" = NONE -o x"$mvapich2_conf_blcr_home" = x ]
				then
					mvapich2_conf_blcr_home=""
				elif [ ! -d "$mvapich2_conf_blcr_home" ]
				then
					echo "$mvapich2_conf_blcr_home: Not Found"
				else
					break
				fi
			else
				mvapich2_conf_ckpt=0
				break
			fi
		done
	fi
fi

if [ "$ARCH" = "PPC64" -a \
	\( ! -f /etc/SuSE-release -o "$dist_rpm_rel_int" -le "1502" \) ]	# eg. 15.2
then
	export LDFLAGS="-m64 -g -O2 -L/usr/lib64 -L/usr/X11R6/lib64"
	export CFLAGS="-m64 -g -O2"
	export CPPFLAGS="-m64 -g -O2"
	export CXXFLAGS="-m64 -g -O2"
	export FFLAGS="-m64 -g -O2"
	export FCFLAGS="-m64 -g -O2"
	export LDLIBS="-m64 -g -O2 -L/usr/lib64 -L/usr/X11R6/lib64"
else
	# just to be safe
	unset LDFLAGS
	unset CFLAGS
	unset CPPFLAGS
	unset CXXFLAGS
	unset FFLAGS
	unset FCFLAGS
	unset LDLIBS
fi

logfile=make.mvapich2.$interface.$compiler
(
	STACK_PREFIX=${STACK_PREFIX:-/usr}
	BUILD_DIR=${BUILD_DIR:-/var/tmp/Intel-mvapich2}
	BUILD_ROOT="$BUILD_DIR/build";
	RPM_DIR="$BUILD_DIR/OFEDRPMS";
	DESTDIR=/usr/src/opa/MPI
	if [ "$iflag" = n ]
	then
	    mvapich2_srpm=/usr/src/opa/MPI/mvapich2-*.src.rpm
	    mpitests_srpm=/usr/src/opa/MPI/mpitests-*.src.rpm
	else
	    mvapich2_srpm=./SRPMS/mvapich2-*.src.rpm
	    mpitests_srpm=./SRPMS/mpitests-*.src.rpm
	fi
	mvapich2_version=$(ls $mvapich2_srpm 2>/dev/null|head -1|cut -f2 -d-)
	mvapich2_fullversion=$(ls $mvapich2_srpm 2>/dev/null|head -1|cut -f2- -d-|sed -e 's/.src.rpm//')
	mpitests_version=$(ls $mpitests_srpm 2>/dev/null|head -1|cut -f2 -d-)
	mpitests_fullversion=$(ls $mpitests_srpm 2>/dev/null|head -1|cut -f2- -d-|sed -e 's/.src.rpm//')
	MPICH_PREFIX=${MPICH_PREFIX:-$STACK_PREFIX/mpi/$compiler/mvapich2-$mvapich2_version$mvapich2_path_suffix}
	CONFIG_OPTIONS=${CONFIG_OPTIONS:-""}

	if [ x"$mvapich2_version" = x"" ]
	then
		echo "Error $mvapich2_srpm: Not Found"
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
	echo "MVAPICH2 Version: $mvapich2_version"
	echo "MVAPICH2 Full Version: $mvapich2_fullversion"
	echo "mpitests Version: $mpitests_version"
	echo "mpitests Full Version: $mpitests_fullversion"
	echo "=========================================================="
	if [ "$iflag" = n ]
	then
		echo "MPICH_PREFIX='$MPICH_PREFIX'"> /usr/src/opa/MPI/.mpiinfo
		#echo "MPI_RUNTIME='$MPICH_PREFIX/bin $MPICH_PREFIX/lib $MPICH_PREFIX/tests'">> /usr/src/opa/MPI/.mpiinfo
		echo "MPI_RPMS='mvapich2_$compiler$mvapich2_rpm_suffix-$mvapich2_fullversion.$target_cpu.rpm mpitests_mvapich2_$compiler$mvapich2_rpm_suffix-$mpitests_fullversion.$target_cpu.rpm'">> /usr/src/opa/MPI/.mpiinfo
		chmod +x /usr/src/opa/MPI/.mpiinfo
	fi

	echo
	echo "Cleaning build tree..."
	rm -rf $BUILD_DIR > /dev/null 2>&1

	echo "=========================================================="
	echo "Building MVAPICH2 MPI $mvapich2_version Library/Tools..."
	mkdir -p $BUILD_ROOT $RPM_DIR/BUILD $RPM_DIR/RPMS $RPM_DIR/SOURCES $RPM_DIR/SPECS $RPM_DIR/SRPMS

	disable_auto_requires=""
	# need to create proper mvapich2_comp_env value for MVAPICH2 builds
	case "$compiler" in
	gcc)
		if have_comp gfortran
		then
			if [ "$ARCH" = "PPC64" ]
			then
				mvapich2_comp_env='CC="gcc -m64" CXX="g++ -m64" F77="gfortran -m64" FC="gfortran -m64"'
			else
				mvapich2_comp_env='CC=gcc CXX=g++ F77=gfortran FC=gfortran'
			fi
		else
			if [ "$ARCH" = "PPC64" ]
			then
				mvapich2_comp_env='CC="gcc -m64" CXX="g++ -m64" F77="g77 -m64" FC="/bin/false"'
			else
				mvapich2_comp_env='CC=gcc CXX=g++ F77=g77 FC=/bin/false'
			fi
		fi;;

	pathscale)
		disable_auto_requires="--define 'disable_auto_requires 1'"
		mvapich2_comp_env='CC=pathcc CXX=pathCC F77=pathf90 FC=pathf90'
		if [ "$target_cpu" = "i686" && $mvapich2_conf_shared_libs  = 1 ]
		then
			# on i686 with shared libs need -g for MVAPICH2
			mvapich2_comp_env="$mvapich2_comp_env OPT_FLAG='-g'"
		fi;;

	pgi)
		disable_auto_requires="--define 'disable_auto_requires 1'"
		mvapich2_comp_env='CC=pgcc CXX=pgCC F77=pgf77 FC=pgf90';;

	intel)
		disable_auto_requires="--define 'disable_auto_requires 1'"
		if [ "$mvapich2_conf_shared_libs" = 1 ] 
		then
			mvapich2_comp_env='CC=icc CXX=icpc F77=ifort FC=ifort'
		else
			mvapich2_comp_env='CC=icc CXX=icpc F77=ifort FC=ifort'
		fi;;

	*)
		echo "ERROR: Invalid compiler"
		exit 1;;
	esac

	pref_env=
	if [ "$STACK_PREFIX" != "/usr" ]
	then
		pref_env="$pref_env LD_LIBRARY_PATH=$STACK_PREFIX/lib64:$STACK_PREFIX/lib:\$LD_LIBRARY_PATH"
	fi

	cmd="$pref_env rpmbuild --rebuild \
				--define '_topdir $RPM_DIR' \
				--buildroot '$BUILD_ROOT' \
				--define 'build_root $BUILD_ROOT' \
				--target $target_cpu \
				--define '_name mvapich2_$compiler$mvapich2_rpm_suffix' \
				--define 'compiler $compiler' \
				--define '$mvapich2_conf_impl_define' \
				--define 'open_ib_home $STACK_PREFIX' \
				--define '_usr $STACK_PREFIX' \
				--define 'comp_env $mvapich2_comp_env $CONFIG_OPTIONS $mvapich2_conf_psm' \
				--define 'auto_req 0' \
				--define 'mpi_selector $STACK_PREFIX/bin/mpi-selector' \
				--define '_prefix $MPICH_PREFIX'"
	#cmd="$cmd --define 'shared_libs $mvapich2_conf_shared_libs'"
	if [ "$mvapich2_conf_shared_libs" = 1 ]
	then
		cmd="$cmd --define 'shared_libs $mvapich2_conf_shared_libs'"
	fi
	#cmd="$cmd --define 'romio $mvapich2_conf_romio'"
	if [ "$mvapich2_conf_romio" = 1 ]
	then
		cmd="$cmd --define 'romio $mvapich2_conf_romio'"
	fi
	if [ "$mvapich2_conf_impl" = "ofa" ]
	then
		cmd="$cmd --define 'rdma --with-rdma=gen2' \
				--define 'ib_include --with-ib-include=$STACK_PREFIX/include'"
		if [ "$ARCH" = "PPC64" -o "$ARCH" = "X86_64" ]
		then
			cmd="$cmd --define 'ib_libpath --with-ib-libpath=$STACK_PREFIX/lib64'"
		else
			cmd="$cmd --define 'ib_libpath --with-ib-libpath=$STACK_PREFIX/lib'"
		fi
		if [ "$mvapich2_conf_ckpt" = 1 ]
		then
			cmd="$cmd --define 'blcr 1'"
			cmd="$cmd --define 'blcr_include --with-blcr-include=$mvapich2_conf_blcr_home/include' \
				--define 'blcr_libpath --with-blcr-libpath=$mvapich2_conf_blcr_home/lib'"
		fi
	elif [ "$mvapich2_conf_impl" = "psm" ]
	then
		# no special args needed
		> /dev/null
	else
		echo "ERROR: Invalid mvapich2_conf_impl: $mvapich2_conf_impl" >&2
		exit 1
	fi

	cmd="$cmd \
				$mvapich2_srpm"
	echo "Executing: $cmd"
	eval $cmd
	if [ $? != 0 ]
	then
		echo "error: mvapich2_$compiler$mvapich2_rpm_suffix Build ERROR: bad exit code"
		exit 1
	fi
	if [ "$iflag" = n ]
	then
		cp $RPM_DIR/RPMS/$target_cpu/mvapich2_$compiler$mvapich2_rpm_suffix-$mvapich2_fullversion.$target_cpu.rpm $DESTDIR
	fi

	echo "=========================================================="
	echo "Installing MVAPICH2 MPI $mvapich2_version Library/Tools..."
	rpmfile=$RPM_DIR/RPMS/$target_cpu/mvapich2_$compiler$mvapich2_rpm_suffix-$mvapich2_fullversion.$target_cpu.rpm

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

	echo "=========================================================="
	echo "Building test programs $mpitests_version for MVAPICH2 MPI $mvapich2_version..."
	# mpitests uses buildroot instead of build_root, play it safe for future
	# and define both
	cmd="$pref_env rpmbuild --rebuild \
				--define '_topdir $RPM_DIR' \
				--buildroot '$BUILD_ROOT' \
				--define 'buildroot $BUILD_ROOT' \
				--define 'build_root $BUILD_ROOT' \
				--target $target_cpu \
				--define '_name mpitests_mvapich2_$compiler$mvapich2_rpm_suffix' \
				--define 'root_path /' \
				--define '_usr $STACK_PREFIX' \
				--define 'path_to_mpihome $MPICH_PREFIX' \
				$disable_auto_requires \
				$mpitests_srpm"
	echo "Executing: $cmd"
	eval $cmd
	if [ $? != 0 ]
	then
		echo "error: mpitests_mvapich2_$compiler$mvapich2_rpm_suffix Build ERROR: bad exit code"
		exit 1
	fi
	
	if [ "$iflag" = n ]
	then
		mv $RPM_DIR/RPMS/$target_cpu/mpitests_mvapich2_$compiler$mvapich2_rpm_suffix-$mpitests_fullversion.$target_cpu.rpm $DESTDIR

		echo "=========================================================="
		echo "Installing test programs $mpitests_version for MVAPICH2 MPI $mvapich2_version..."
		rpmfile=$DESTDIR/mpitests_mvapich2_$compiler$mvapich2_rpm_suffix-$mpitests_fullversion.$target_cpu.rpm
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

if egrep 'error:|Error | Stop' $logfile.res > $logfile.err
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
