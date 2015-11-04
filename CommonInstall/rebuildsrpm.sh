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

# run this script from within the IntelOPA-Basic/*OFED* 
# tree which is patched.  It will recreate the ofa_kernel SRPMS
# it expects the ofa_kernel srpm to have been torn apart via:
#	cd *OFED*
#	mkdir patched
#	cd patched
#	rpm2cpio ../ofa_kernel*|cpi -icvd
#	tar xvfz *.tgz
#	edit source files as needed

# this is strictly for internal use by development, not included in product
# however it is based on similar algorithms for reassemblind srpms which
# are part of scripts such as patch_ofed3.sh

rebuild_srpm()
{
	# $1 of -R will reverse patch
	# $1 of -C will remove all signs we patched it
	clean=
	reverse=
	if [ "x$1" = "x-R" ]
	then
		reverse=$1
	elif [ "x$1" = "x-C" ]
	then
		clean=y
	fi 

	# this will force rebuild from srpm
	rm -f RPMS/*/kernel-ib-1*.rpm

	cd SRPMS
	srpm=`ls ofa_kernel-1*.src.rpm`
	if [ ! -e $srpm-save ]
	then
		mv $srpm $srpm-save
	else
		if [ "x$reverse" = "x-R" ]
		then
			cp $srpm-save $srpm
			return
		else
			rm $srpm
		fi
	fi
	cd patched
	tgz=`ls *.tgz`
	base=`basename $tgz .tgz`
	tar cfz $tgz $base
	
	# regenerate the patched srpm
	rm -rf SOURCES SPECS SRPMS
	mkdir SOURCES SPECS SRPMS
	cp $tgz SOURCES/
	cp *.spec SPECS/
	rpmbuild -bs --define "_topdir `pwd`" SPECS/*.spec
	cp SRPMS/*.src.rpm ../
	cd ..
	if [ "x$clean" = "xy" ]
	then
		rm -rf $srpm-save
	fi
}

if [ -d ../*OFED* ]
then
	if [ ! -d SRPMS/patched ]
	then
		echo "the patched source must be in SRPMS/patched"
		exit 1
	fi
	rebuild_srpm $1
	echo "This ofa_kernel srpm has been rebuild and may now be installed"
else
	echo "This must be run within the IntelOPA-*/*OFED* directory" 2>/dev/null
fi
