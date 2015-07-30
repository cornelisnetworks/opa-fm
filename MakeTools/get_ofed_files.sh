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

#[ICS VERSION STRING: unknown]
# this script builds a tgz with all the header and library files needed
# in order to build applications against a new version of OFED
# this should be run on each distro.  It uninstalls OFED, gets a file list for
# /usr, then builds and installs ofed and builds the list of new files
# which came from OFED.  Those files are tared up and made ready to be
# put in the /usr/ofed-$version directory on the given build or sds machine

if [ $# -ne 1 ]
then
	echo "Usage: get_ofed_files.sh version" >&2
	exit 2
fi
version=$1

set -x
dir=$PWD

# uninstall any QLogic code
opaconfig -u

# untar OFED installer
rm -rf OFED-$version
#tar xvfz OFED-$version.tar
tar xvfz OFED-$version.tgz
cd OFED-$version

# uninstall OFED
echo y|./uninstall.sh

# list of files before OFED
find /usr > $dir/usr.before

# install all of OFED
./install.pl --all

# list of files after OFED
cd $dir
find /usr > $dir/usr.after

# figure out what OFED added in /usr
diff $dir/usr.before $dir/usr.after > $dir/usr.ofed

# prune out some directories which do not have headers nor libraries
sed -e '/^[0-9]/d' -e 's/^> //' -e '/^< /d' -e '/^---/d' -e '/^\/usr\/sbin/d' -e '/^\/usr\/bin/d' -e '/^\/usr\/src/d' -e '/^\/usr\/mpi/d' -e '/^\/usr\/share\/man/d' -e 's/^\/usr\///' < $dir/usr.ofed > $dir/usr.ofed.files

# tar up the files of interest
cd /usr
tar cvfz $dir/ofed-$version.tgz -T $dir/usr.ofed.files
