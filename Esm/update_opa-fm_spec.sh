#!/bin/bash

id=$(grep ^ID= /etc/os-release | cut -f2 -d\")
versionid=$(grep ^VERSION_ID= /etc/os-release | cut -f2 -d\")

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

if [ "$id" = "rhel" ]
then
	sed -i "s/__RPM_BLDRQ1/expat-devel, libibumad-devel, libibverbs-devel, libibmad-devel, openssl-devel/g" $to
	st=$(echo "$versionid >= 7.0" | bc)
	if [ $st = 1 ]
	then
		sed -i "s/__RPM_BLDRQ2/BuildRequires: systemd %{?systemd_requires} %{?BuildRequires}/g" $to
		sed -i "s/__RPM_RQ1/Requires: systemd %{?systemd_requires}/g" $to
		sed -i "/__RPM_SYSCONF/,+1d" $to
	else
		sed -i "s/__RPM_BLDRQ2/Requires(post): \/sbin\/chkconfig/g" $to
		sed -i "s/__RPM_RQ1/Requires(preun): \/sbin\/chkconfig/g" $to
		sed -i "s/__RPM_INS/install -D -m 755 stage.rpm\/opafm $RPM_BUILD_ROOT%{_sysconfdir}\/init.d\/opafm/g" $to
		sed -i "s/__RPM_SYSCONF/%{_sysconfdir}\/init.d\/opafm/g" $to
	fi
	sed -i "s/__RPM_RQ2/Requires: libibumad%{?_isa}, libibmad%{?_isa}, libibverbs%{?_isa}, rdma, expat%{?_isa}, libhfi1, openssl%{?_isa}/g" $to
	sed -i "/__RPM_DEBUG/,+1d" $to
elif [ "$id" = "sles" ]
then
	sed -i "s/__RPM_BLDRQ1/libexpat-devel, libibumad-devel, libibverbs-devel, libibmad-devel, openssl-devel/g" $to
	st=$(echo "$versionid >= 12.1" | bc)
	if [ $st = 1 ]
	then
		sed -i "s/__RPM_BLDRQ2/BuildRequires: systemd %{?systemd_requires} %{?BuildRequires}/g" $to
		sed -i "s/__RPM_RQ1/Requires: systemd %{?systemd_requires}/g" $to
	else
		sed -i "s/__RPM_BLDRQ2/Requires(post): /sbin/chkconfig/g" $to
		sed -i "s/__RPM_RQ1/Requires(preun): /sbin/chkconfig/g" $to
	fi
	st=$(echo "$versionid >= 11.1" | bc)
	if [ $st = 1 ]
	then
		sed -i "s/__RPM_DEBUG/%debug_package/g" $to
	else
		sed -i "/__RPM_DEBUG/,+1d" $to
	fi
	sed -i "/__RPM_INS/,+1d" $to
	sed -i "/__RPM_SYSCONF/,+1d" $to
	sed -i "s/__RPM_RQ2/Requires: libibumad3, libibmad5, libibverbs1, rdma, libexpat1, openssl/g" $to
else
	echo ERROR: Unsupported distribution: $id $versionid
	exit 1
fi

exit 0



















