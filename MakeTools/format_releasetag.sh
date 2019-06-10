#!/bin/bash
# BEGIN_ICS_COPYRIGHT8 ****************************************
# 
# Copyright (c) 2015-2019, Intel Corporation
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


# Parse release tags to create version strings
# - X.Y.m.p.b  - a build on the way to final
# - X.Y.mAn.b – Alpha “n” build  (sometimes called release candidate in redhat terminology)
# - X.Y.mBn.b – Beta “n” build  (sometimes called release candidate in redhat terminology)
# - X.Y.mRCn.b – final release candidate “n” build
# - X.Y.m.0.b – a final PV release
# - X.Y.m.p.b – a patch of a PV release
#
# 32-bit hex (UEFI)
# - X – 4 bits
# - Y - 8 bits
# - m – 4 bits
#   quality – 4 bits (0-2=power on, 3-5 = Alpha, 6-8=Beta, 9-13=release candidates, 14=PRQ/PV, 15=patch)
#   for quality=0-14: b – 12 bits
#   when quality==15: p-4 bits, b - 8 bits [unlikely to be used, but covers in case we need it]
#
# 64-bit hex (switch firmware)
# - X – 8 bits
# - Y - 8 bits
# - m – 8 bits
# - quality – 4 bit (0-2=power on, 3-5 = Alpha, 6-8=Beta, 9-13=release candidates, 14=PRQ/PV or  patch)
# - patch – 4 bits
# - build – 32 bits
#
# Also allow for user-named version of format 0<username>



checkAlpha()
{
	if [ $user_ver -eq 1 ]
	then
		return 0
	fi

	pos=$1
	ver=$2

	if [ "$ver" = "" ]
	then
		return 0
	fi

	if echo $ver | grep -qe [[:alpha:]]
	then
		if echo $ver | grep -qv I
		then
			echo Error: invalid alpha character in $pos: $ver
			return 1
		fi
	fi
	return 0
}

parseOutI()
{
	ver=$1

	iP=$(echo $ver | grep -bo I | head -n 1 | cut -d: -f1)
	v1=${ver:0:iP}
	echo $v1
}

parseBldI()
{
	ver=$1

	iP=$(echo $ver | grep -bo I | head -n 1 | cut -d: -f1)
	bld=${ver:$((iP + 1))}
	echo $bld
}

getQual()
{
	qual=$1
	pat=$2
	n=$3

	if [ $pat -ne 0 ]
	then
		echo 15
	else
		case $qual in
			poweron)
				if [ $n -gt 2 ]
				then
					(>&2 echo "Warning: Encoded Quality out of range: $n Clamping to: 2")
					n=2
				fi
				echo $n
				;;
			alpha)
				n=$((3 + n))
				if [ $n -gt 5 ]
				then
					(>&2 echo "Warning: Encoded Quality out of range: $n Clamping to: 5")
					n=5
				fi
				echo $n
				;;
			beta)
				n=$((6 + n))
				if [ $n -gt 8 ]
				then
					(>&2 echo "Warning: Encoded Quality out of range: $n Clamping to: 8")
					n=8
				fi
				echo $n
				;;
			rc)
				n=$((9 + n))
				if [ $n -gt 13 ]
				then
					(>&2 echo "Warning: Encoded Quality out of range: $n Clamping to: 13")
					n=13
				fi
				echo $n
				;;
			pv)
				echo 14
				;;
		esac
	fi
	return
}

if [ "$#" -eq 0 ]
then
	echo Error: too few arguments
	echo Usage: format_releasetag.sh [-f format] version-string
	exit 1
fi

if [ "$1" = "" ]
then
	echo Error: invalid version string argument
	echo Usage: format_releasetag.sh [-f format] version-string
	exit 1
fi

fmt=
ver=

while getopts "f:" opt
do
	case "$opt" in
	f)
		fmt=$OPTARG
		;;
	?)
		echo Error: unsupported option $opt
		echo Usage: format_releasetag.sh [-f format] version-string
		exit 1
		;;
	esac
done

shift $((OPTIND-1))

ver=$1

# check for user-named release tag - leading 0 followed by alpha

user_ver=0
if echo ${ver:0:1} | grep -q 0
then
	if echo ${ver:1:1} | grep -q '[[:alpha:]]'
	then
		user_ver=1
	fi
fi

if [ "$2" != "" ]
then
	echo Error: extra arguments
	echo Usage: format_releasetag.sh [-f format] version-string
	exit 1
fi

if [ "$fmt" = "" ]
then
	fmt=text
fi

if echo $ver | grep -q '[^ A-Za-z0-9_.]'
then
	echo Error: bad format of version $ver
	exit 1
fi

# strip off leading 'R'

if echo ${ver:0:1} | grep -q R
then
	ver=${ver:1}
fi

# get to all underscores and parse

newver=$(echo $ver | sed 's/\./_/g')
numseparators=$(echo $newver | grep -bo '_'|wc -l)
maj=$(echo $newver | cut -d_ -f1)
if [ $numseparators -gt 0 ]
then
	min=$(echo $newver | cut -d_ -f2)
else
	min=0
fi
if [ $numseparators -gt 1 ]
then
	mai=$(echo $newver | cut -d_ -f3)
else
	mai=0
fi
if [ $numseparators -gt 2 ]
then
	pat=$(echo $newver | cut -d_ -f4)
else
	pat=0
fi
if [ $numseparators -gt 3 ]
then
	bld=$(echo $newver | cut -d_ -f5)
else
	bld=0
fi

# alpha only allowed in $mai
checkAlpha major "$maj" || exit 1
checkAlpha minor "$min" || exit 1
checkAlpha patch "$pat" || exit 1
checkAlpha build "$bld" || exit 1

# Handle no mai/pat/bld

if [ "$mai" = "" ]
then
	mai=0
fi
if [ "$pat" = "" ]
then
	pat=0
fi
if [ "$bld" = "" ]
then
	bld=0
fi

# handle I format

if echo $maj | grep -q I
then
	bld=$(parseBldI $maj)
	maj=$(parseOutI $maj)
fi
if echo $min | grep -q I
then
	bld=$(parseBldI $min)
	min=$(parseOutI $min)
fi
if echo $mai | grep -q I
then
	bld=$(parseBldI $mai)
	mai=$(parseOutI $mai)
fi
if echo $pat | grep -q I
then
	bld=$(parseBldI $pat)
	pat=$(parseOutI $pat)
fi

# parse maintenance in the form m<alpha>n

mai_alpha=0

m=
n=
alpha=PV
if echo $mai | grep -q [[:alpha:]]
then
	mai_alpha=1
	pat=0
	alphaP=$(echo $mai | grep -bo '[[:alpha:]]'|head -n 1|cut -d: -f1)
	nP1=$(echo ${mai:$alphaP} | grep -bo '[[:digit:]]'|head -n 1|cut -d: -f1)
	nP=$((alphaP + nP1))
	m=${mai:0:alphaP}
	alpha=${mai:alphaP:$((dP-alphaP))}
	n=${mai:$nP}
	bld=$(echo $newver | cut -d_ -f4)
	if echo $alpha | grep -q I
	then
		iformat=1
	else
		iformat=0
	fi
	if echo $alpha | grep -q P
	then
		qual=poweron
	elif echo $alpha | grep -q A
	then
		qual=alpha
		alpha=$(echo $alpha | tr '[:upper:]' '[:lower:]')
	elif echo $alpha | grep -q B
	then
		qual=beta
		alpha=$(echo $alpha | tr '[:upper:]' '[:lower:]')
	elif echo $alpha | grep -q RC
	then
		qual=rc
		alpha=$(echo $alpha | tr '[:upper:]' '[:lower:]')
	elif echo $alpha | grep -q PV
	then
		qual=pv
	elif [ $user_ver -eq 1 ]
	then
		qual=pv
	elif [ $iformat -eq 0 ]
	then
		echo Error: Unknown release tag quality: $alpha
		exit 1
	fi
	if [ $iformat -eq 1 ]
	then
		iP=$(echo $mai | grep -bo I |head -n 1|cut -d: -f1)
		bld=${mai:$((iP+1))}
	else
		bld=$(echo $newver | cut -d_ -f4)
	fi
else
	qual=pv
fi


case $fmt in
	text)
		if [ $user_ver -eq 1 ]
		then
			echo $ver
			exit 0
		fi
		q=$(getQual $qual $pat $n)
		if [ $mai_alpha -eq 1 ]
		then
			echo ${maj}.${min}.${mai}.${bld}
		else
			echo ${maj}.${min}.${mai}.${pat}.${bld}
		fi
		;;
	rpm)
		if [ $user_ver -eq 1 ]
		then
			echo $ver
			exit 0
		fi
		q=$(getQual $qual $pat $n)
		if [ $mai_alpha -eq 1 ]
		then
			echo ${maj}.${min}.${m}.0-${alpha}${n}.${bld}
		else
			echo ${maj}.${min}.${mai}.${pat}-${bld}
		fi
		;;
	briefrpm)
		if [ $user_ver -eq 1 ]
		then
			echo $ver
			exit 0
		fi
		if [ $mai_alpha -eq 1 ]
		then
			m1=$m
		else
			m1=$mai
		fi
		if [ $pat -gt 0 ]
		then
			maint=".${m1}.${pat}"
		elif [ $m1 -gt 0 ]
		then
			maint=".${m1}"
		else
			maint=
		fi
		q=$(getQual $qual $pat $n)
		if [ $mai_alpha -eq 1 ]
		then
			echo ${maj}.${min}${maint}-${alpha}${n}.${bld}
		else
			echo ${maj}.${min}${maint}-${bld}
		fi
		;;
	debian)
		if [ $user_ver -eq 1 ]
		then
			echo $ver
			exit 0
		fi
		q=$(getQual $qual $pat $n)
		if [ $mai_alpha -eq 1 ]
		then
			echo ${maj}.${min}.${m}.0${alpha}${n}.${bld}
		else
			echo ${maj}.${min}.${mai}.${pat}.${bld}
		fi
		;;
	32bit)
		if [ $user_ver -eq 1 ]
		then
			echo 0x00000000
			exit 0
		fi
		if [ $mai_alpha -eq 1 ]
		then
			m1=$m
		else
			m1=$mai
		fi
		byte1=$((maj<<4 | min&0xf0))
		byte2=$(((min & 0x0f)<<4 | (m1 & 0x0f)))
		q=$(getQual $qual $pat $n)
		if [ $pat -gt 0 ]
		then
			byte3=$((q<<4 | (pat & 0xff)))
		else
			bldHiBits=$(((bld & 0xf00) >> 8))
			byte3=$((q<<4 | bldHiBits))
		fi
		byte4=$((bld & 0xff))
		printf "0x%02X%02X%02X%02X\n" $byte1 $byte2 $byte3 $byte4
		;;
	64bit)
		if [ $user_ver -eq 1 ]
		then
			echo 0x0000000000000000
			exit 0
		fi
		byte1=${maj}
		byte2=${min}
		if [ $mai_alpha -eq 1 ]
		then
			m1=$m
		else
			m1=$mai
		fi
		byte3=${m1}
		if [ $pat -gt 0 ]
		then
			q=14
		else
			q=$(getQual $qual $pat $n)
		fi
		byte4=$((q<<4 | (pat & 0xff)))
		bytes5to8=$bld
		printf "0x%02X%02X%02X%02X%08X\n" $byte1 $byte2 $byte3 $byte4 $bytes5to8
		;;
	?)
		echo Error: bad format: $fmt
		exit 1
		;;
esac


exit 0
