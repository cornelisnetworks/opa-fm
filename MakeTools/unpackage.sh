#/bin/sh
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

#set -x

if [ -d src ]
then
	rm -rf src
fi
mkdir src

if [ -f cvsignores.tgz ]
then
	rm -f cvsignores.tgz
fi

rm -rf SOURCES/*
rm -rf SPECS/*

for x in SRPMS/*.src.rpm;
do
	SRC_RPM=${x##*/}
	PACKAGE_DIR=${SRC_RPM/.src.rpm}
	
	echo "Installing $SRC_RPM"
	
	rpm -i --define '_topdir '$PWD'' $x 2> /dev/null
	
	mkdir src/$PACKAGE_DIR;
	
	for y in SPECS/*;
	do
		mv $y src/$PACKAGE_DIR
	done
	
	for y in SOURCES/*;
	do
		mv $y src/$PACKAGE_DIR
		pushd src/$PACKAGE_DIR 1> /dev/null
		
		for z in *;
		do
			if [ ${z##*.} == "gz" -o ${z##*.} == "tgz" ]
			then
				tar xzf $z
			else
				if [ ${z##*.} == "bz2" ]
				then
					tar xjf $z
				fi
			fi
			
		done
		
		popd 1> /dev/null
	done
done

find ./src -name .cvsignore | xargs tar -c -z --remove-files -f cvsignores.tgz

for x in src/*;
do
	BASE_DIR="src/ofa_kernel"
	if [ ${x:0:${#BASE_DIR}} == $BASE_DIR ]
	then
		for y in $x/*
		do
			if [ -d $y ]
			then
				pushd $y 1> /dev/null
				rm -f configure Makefile makefile
				cp ofed_scripts/configure .
				cp ofed_scripts/Makefile .
				cp ofed_scripts/makefile .
				pushd drivers/scsi 1> /dev/null
				rm Makefile
				cp ../../ofed_scripts/iscsi_scsi_makefile Makefile
				popd +2 1> /dev/null
				break
			fi
		done
		break
	fi
done
