#!/bin/bash
# BEGIN_ICS_COPYRIGHT8 ****************************************
# 
# Copyright (c) 2018, Intel Corporation
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

# rebuild selected MPI to target a specific compiler

Usage()
{
	echo "Usage: do_build [-d] [-Q] [mpi [config_opt [install_dir]]]" >&2
	echo "        -d - use default settings for selected MPI's options" >&2
	echo "            if omitted, will be prompted for each option" >&2
	echo "        -Q - build the MPI targeted for the PSM API." >&2
	echo "        mpi - MPI to build (mvapich, mvapich2 or openmpi)" >&2
	echo "        config_opt - a compiler selection option (gcc, pathscale, pgi or intel)" >&2
	echo "             if config_opt is not specified, the user will be prompted" >&2
	echo "             based on compilers found on this system" >&2
	echo "        install_dir - where to install MPI, see MPICH_PREFIX below" >&2
	echo "" >&2
	echo "Environment:" >&2
	echo "    STACK_PREFIX - where to find IB stack." >&2
	echo "    BUILD_DIR - temporary directory to use during build of MPI" >&2
	echo "            Default is /var/tmp/Intel-mvapich or /var/tmp/Intel-mvapich2" >&2
	echo "            or /var/tmp/Intel-openmpi" >&2
	echo "    MPICH_PREFIX - selects location for installed MPI" >&2
	echo "            default is /usr/mpi/<COMPILER>/<MPI>-<VERSION>" >&2
	echo "            where COMPILER is selected compiler (gcc, pathscale, etc above)" >&2
	echo "            MPI is mvapich, mvapich2 or openmpi" >&2
	echo "            and VERSION is MPI version (eg. 1.0.0)" >&2
	echo "    CONFIG_OPTIONS - additional configuration options to be" >&2
	echo "            specified to srpm (not applicable to mvapich)" >&2
	echo "            Default is ''" >&2
	echo "    INSTALL_ROOT - location of system image in which to install." >&2
	echo "            Default is '/'" >&2
	echo "" >&2
	echo "The RPMs built during this process will be installed on this system" >&2
	echo "they can also be found in /usr/src/opa/MPI" >&2
	exit 2
}

build_opts=
while getopts "Qd" o
do
	case "$o" in
	d) build_opts="$build_opts -d";;
	Q) build_opts="$build_opts -Q";;
	*) Usage;;
	esac
done
shift $((OPTIND -1))
if [ $# -gt 3 ]
then
	Usage
fi

if [ "$(/usr/bin/id -u)" != 0 ]
then
	echo "You must be 'root' to run this program" >&2
	exit 1
fi
cd /usr/src/opa/MPI
if [ $? != 0 ]
then
	echo "Unable to cd to /usr/src/opa/MPI" >&2
	exit 1
fi

echo
echo "IFS MPI Library/Tools rebuild"
mpi="$1"
compiler="$2"
if [ ! -z "$3" ]
then
	export MPICH_PREFIX="$3"
fi

nompi()
{
	echo "No MPI Build scripts Found, unable to Rebuild MPI" >&2
	exit 1
}

if [ -z "$mpi" ]
then
	choices=""
	for i in openmpi mvapich2
	do
		if [ -e do_${i}_build ]
		then
			choices="$choices $i"
		fi
	done
	if [ x"$choices" = x ]
	then
		nompi
	else
		PS3="Select MPI to Build: "
		select mpi in $choices
		do
			case "$mpi" in
			mvapich2|openmpi) break;;
			esac
		done
	fi
fi

./do_${mpi}_build $build_opts "$compiler"
exit $?
