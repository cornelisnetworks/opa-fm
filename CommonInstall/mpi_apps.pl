#!/usr/bin/perl
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

# [ICS VERSION STRING: unknown]
use strict;
#use Term::ANSIColor;
#use Term::ANSIColor qw(:constants);
#use File::Basename;
#use Math::BigInt;


# ==========================================================================
# SHMEM Sample Applications sub-component installation

# remove the files built from source or installed
# (as listed in .files and .dirs)
# $0 = dirname relative to $ROOT
sub remove_shmem_apps($)
{
	my $dirname = shift();

	if ( -d "$ROOT/$dirname" ) {
		system "cd $ROOT/$dirname; make clean >/dev/null 2>&1";
		system "cd $ROOT/$dirname; cat .files 2>/dev/null|xargs rm -f 2>/dev/null";
		system "cd $ROOT/$dirname; cat .files 2>/dev/null|sort -r|xargs rmdir 2>/dev/null";
		system "rm -f $ROOT/$dirname/.files 2>/dev/null";
		system "rmdir $ROOT/$dirname/ 2>/dev/null";
	}
}

sub uninstall_shmem_apps();

sub install_shmem_apps($)
{
	my $srcdir=shift();

	my $file;

	# clean existing shmem sample applications
	uninstall_shmem_apps;

	# Copy all shmem sample applications
	check_dir("/usr/src/opa");
	check_dir("/usr/src/opa/shmem_apps");
	if ( -e "$srcdir/shmem/shmem_apps/shmem_apps.tgz" )
	{
		system("tar xfvz $srcdir/shmem/shmem_apps/shmem_apps.tgz --directory $ROOT/usr/src/opa/shmem_apps > $ROOT/usr/src/opa/shmem_apps/.files");
	}
	# allow all users to read the files so they can copy and use
	system("chmod -R ugo+r $ROOT/usr/src/opa/shmem_apps");
	system("find $ROOT/usr/src/opa/shmem_apps -type d|xargs chmod ugo+x");
}

sub uninstall_shmem_apps()
{
	# remove shmem_apps we installed or user compiled, however do not remove
	# any logs or other files the user may have created
	remove_shmem_apps "/usr/src/opa/shmem_apps";
	system "rmdir $ROOT/usr/src/opa 2>/dev/null";	# remove only if empty
	system "rmdir $ROOT/usr/lib/opa 2>/dev/null";	# remove only if empty
}

