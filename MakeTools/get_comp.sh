#!/bin/bash
# BEGIN_ICS_COPYRIGHT8 ****************************************
#
# Copyright (c) 2015-2020, Intel Corporation
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
##
## get_comp
## -----------
## get a component which will be wrapped into the install wrapper
## the component is fetched from the ftp server based on the locations
## and files identified by get_input_comp_files based on the WrappedProducts
## configuration script.
##
## Usage:
##	get_comp component bundle downloadsdir [comp.pl ...]
## Arguments:
##	component - one of OFED, SRP, VNIC, FF, FV or FM
##	bundle - one of Basic or IFS or TestTools or TESTS, PRs or RELEASE_NOTES
##	downloadsdir - intermediate local directory to put
##		component files will be downloaded into downloadsdir/component-bundle
##		downloadsdir will be created if needed
##  comp.pl - files to cat together to form comp.pl if product is NONE
## Required Environment (as set via setenv or similar tools):
##	TL_DIR
## 	BUILD_PLATFORM_OS_VENDOR_VERSION
##	BUILD_TARGET
##	BUILD_TARGET_OS_VENDOR
##
## Additional Environment:
##
##	FTPSERVER - ftp server to get files from of the form:
## 			ftp:password@server
##     defaults to: ftp:ftp@kop-sds-ftp
##  if FTPSERVER is cp:self@localhost the directory name for the source file
##	of the component is ignored.  In its place the directory name part of
##	FTPINT is used.  Note that typically FTPSERVER is derrived from FTPINT
##	hence an FTPINT of cp:self@localhost:mydir will do a cp from mydir
##  instead of an ftp.
##
##  SCPSERVER - host to get packages from via SCP. Takes the form:
##	"server" or "user@server:. Defaults to "phcvs2@phlsvlogin02"
##
##  SCPTOP - root directory to get SCP files from. Takes the form:
##  "directory". Defaults to "/nfs/site/proj/stlbuilds".
##
## This is designed for use in higher level scripts and makefiles, as such it
## will only download files which have not already been downloaded.
## It is expected that the higher level script or the makefile's clobber
## removes components-bundle directories in downloadsdir when it wants to
## force this to re-download

. $ICSBIN/funcs.sh

Usage()
{
	# include "ERROR" in message so weeklybuild catches it in error log
	echo "ERROR: get_comp failed" >&2
	echo "Usage: get_comp component bundle downloadsdir [comp.pl ...]" >&2
	exit 2
}

if [ $# -lt 3 ]
then
		Usage
fi

set -e	# exit on any error returns

# $1 = Component Product Name
# $2 = Packaging Bundle
component=$1
bundle=$2
downloadsdir=$3
shift 3


# These variables control where the pre-built packages are pulled from.
# See documentation above.
export FTPSERVER=${FTPSERVER:-"ftp:ftp@kop-sds-ftp"}
export SCPTOP=${SCPTOP:-"/nfs/site/proj/stlbuilds"}
export SCPSERVER=${SCPSERVER:-"phcvs2@awylogin01.aw.intel.com"}:$SCPTOP

# ftp_get_files is defined in newer versions of devtools
if ! type ftp_get_files > /dev/null 2>/dev/null
then
#---------------------------------------------------------------------------
# Function:	ftp_get_files
# In Script:	funcs.sh
# Arguments:	$1 - ftp login and top level dir (user:password@server:dir)
#				$2 - local directory to get files to
#				$3 ...  - list of files to get to current directory
# Description:	copy a set of binary files from the ftp server
# example: ftp_files ftp:password@ftpserver:Integration/GA3_2_0_0 . myfile
#---------------------------------------------------------------------------
function ftp_get_files()
{
	local user password dir server tempdir ldir files

	user=$(echo $1|cut -f1 -d@)
	password=$(echo $user|cut -f2 -d:)
	user=$(echo $user|cut -f1 -d:)
	server=$(echo $1|cut -f2 -d@)
	dir=$(echo $server|cut -f2 -d:)
	server=$(echo $server|cut -f1 -d:)
	ldir=$2
	shift; shift

	files="$*"
	echo "Copying $files from $dir on ftp server $server as User $user..."
	ftp -n $server << EOF
user $user $password
cd $dir
lcd $ldir
binary
prompt
mget $files
quit
EOF
} #end function
fi

function get_file()
{
	# $1 - file to get
	# $2 - directory to put it
	local srcfile file srcdir destdir temp

	srcfile=$1
	destdir=$2
	srcdir=$(dirname $srcfile)
	file=$(basename $srcfile)

	if [ "$FTPSERVER" = "cp:self@localhost" ]
	then
		temp=$(echo $FTPINT|cut -f2 -d@)
		srcdir=$(echo $temp|cut -f2 -d:)
		echo "Copying $srcdir/$file to $destdir/$file..."
		cp $srcdir/$file $destdir/$file
	else
		if [ -d $SCPTOP ]
		then
			echo "Copying $srcfile from $SCPTOP/$srcdir to $destdir/$file..."
			cp $SCPTOP/$srcfile $destdir
		else
			echo "Getting $srcfile from scp: $SCPSERVER:$srcfile to $destdir..."
			scp_get_files $SCPSERVER $srcfile $destdir
		fi
	fi

}

retcode=0

localdir=$downloadsdir/$component-$bundle
mkdir -p $localdir
> $localdir/filelist
files=$($TL_DIR/MakeTools/get_input_comp_files $component $bundle)
if [ $? != 0 ]
then
	echo "ERROR: get_input_comp failed" >&2
	exit 1
fi
echo "$component $bundle Files: $files"
for file in $files
do
	if [ "$file" != "NONE" ]
	then
		echo "$(basename $file)" >> $localdir/filelist
		localfile=$localdir/$(basename $file)
		# get $file from ftp server to $localdir
		get_file $file $localdir
		if [ ! -e $localfile ]
		then
			echo "ERROR - Unable to download $localfile"
			retcode=1
			continue
		fi

		# extract comp.pl file from any .tgz files with one
		case $localfile in
		*.tgz)
			if comp=$(tar tfz $localfile 2>/dev/null | grep '/comp.pl$')
			then
				temp_dir=$localdir/temp
				mkdir $temp_dir
				tar xvfz $localfile -C $temp_dir $comp
			   	mv $temp_dir/*/comp.pl $localdir
				rm -rf $temp_dir
			fi;;
		esac
	elif [ $# != 0 ]
	then
		cat $* > $localdir/comp.pl
	fi
done
if [ $retcode = 0 ]
then
	echo "$(date): $files" > $localdir/download_done
fi

exit $retcode
