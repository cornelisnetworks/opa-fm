#!/bin/bash
# BEGIN_ICS_COPYRIGHT8 ****************************************
# 
# Copyright (c) 2015-2017, Intel Corporation
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

id=$(./Esm/get_id_and_versionid.sh | cut -f1 -d' ')
versionid=$(./Esm/get_id_and_versionid.sh | cut -f2 -d' ')

from=$1
to=$2

if [ "$from" = "" -o "$to" = "" ]
then
	echo "Usage: update_opa-ff_spec.sh spec-in-file spec-file"
	exit 1
fi

if [ "$from" != "$to" ]
then
	cp $from $to
fi

if [ "$id" = "rhel" -o "$id" = "centos" ]
then
	GE_7_0=$(echo "$versionid >= 7.0" | bc)
	GE_7_4=$(echo "$versionid >= 7.4" | bc)
	if [ $GE_7_4 = 1 ]
	then
		sed -i "s/__RPM_BLDRQ1/expat-devel, rdma-core-devel, openssl-devel/g" $to
	else
		sed -i "s/__RPM_BLDRQ1/expat-devel, libibumad-devel, libibverbs-devel, openssl-devel/g" $to
	fi
	if [ $GE_7_0 = 1 ]
	then
		sed -i "s/__RPM_BLDRQ2/BuildRequires: systemd %{?systemd_requires} %{?BuildRequires}/g" $to
		sed -i "s/__RPM_RQ1/Requires: systemd %{?systemd_requires}/g" $to
		sed -i "/__RPM_SYSCONF/,+1d" $to
	else
		sed -i "s/__RPM_BLDRQ2/Requires(post): \/sbin\/chkconfig/g" $to
		sed -i "s/__RPM_RQ1/Requires(preun): \/sbin\/chkconfig/g" $to
		sed -i 's/RPM_INS=n/RPM_INS=y/g' $to
		sed -i "s/__RPM_SYSCONF/%{_sysconfdir}\/init.d\/opafm/g" $to
	fi
	sed -i "s/__RPM_RQ2/Requires: libibumad%{?_isa}, libibmad%{?_isa}, libibverbs%{?_isa}, rdma, expat%{?_isa}, libhfi1, openssl%{?_isa}/g" $to
	sed -i "/__RPM_DEBUG/,+1d" $to
elif [ "$id" = "sles" ]
then
	GE_11_1=$(echo "$versionid >= 11.1" | bc)
	GE_12_1=$(echo "$versionid >= 12.1" | bc)
	GE_12_3=$(echo "$versionid >= 12.3" | bc)
	if [ $GE_12_3 = 1 ]
	then
		sed -i "s/__RPM_BLDRQ1/libexpat-devel, rdma-core-devel, openssl-devel/g" $to
	else
		sed -i "s/__RPM_BLDRQ1/libexpat-devel, libibumad-devel, libibverbs-devel, openssl-devel/g" $to
	fi
	if [ $GE_12_1 = 1 ]
	then
		sed -i "s/__RPM_BLDRQ2/BuildRequires: systemd %{?systemd_requires} %{?BuildRequires}/g" $to
		sed -i "s/__RPM_RQ1/Requires: systemd %{?systemd_requires}/g" $to
	else
		sed -i "s/__RPM_BLDRQ2/Requires(post): /sbin/chkconfig/g" $to
		sed -i "s/__RPM_RQ1/Requires(preun): /sbin/chkconfig/g" $to
	fi
	if [ $GE_11_1 = 1 ]
	then
		sed -i "s/__RPM_DEBUG/%debug_package/g" $to
	else
		sed -i "/__RPM_DEBUG/,+1d" $to
	fi
	sed -i "/__RPM_INS/,+1d" $to
	sed -i "/__RPM_SYSCONF/,+1d" $to
	sed -i "s/__RPM_RQ2/Requires: libibumad3, libibverbs1, rdma, libexpat1, openssl/g" $to
else
	echo ERROR: Unsupported distribution: $id $versionid
	exit 1
fi

exit 0
