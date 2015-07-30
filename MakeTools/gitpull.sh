#!/bin/bash
# github servers where WFR repos are found
server1=git-amr-1.devtools.intel.com:29418
server2=git-amr-2.devtools.intel.com:29418

Usage()
{
	echo "Usage: gitpull.sh [-PSdcv] [-b branch] [-t topdir] [-g gitdir] [repo ...]" >&2
	echo "      -P - skip pulling/cloning from repro" >&2
	echo "      -S - skip building and expanding the srpm" >&2
	echo "      -d - diff against CVS" >&2
	echo "      -c - copy to CVS" >&2
	echo "      -v - verbose output" >&2
	echo "      -b branch - git branch to checkout" >&2
	echo "                  (by default will read branch name from CVS commitids)" >&2
	echo "      -t topdir - top directory in CVS tree (default is $PWD)" >&2
	echo "      -g gitdir - parent directory for git repo(s) (defailt is GIT_REPOS)" >&2
	echo "      repo - list of wfr repos to operate on, default is all repos" >&2
	exit 2
}

# a few aren't working yet
repos="wfr-driver wfr-firmware wfr-psm wfr-diagtools-sw"
repos="$repos wfr-lite wfr-ibacm wfr-libverbs"
repos="$repos wfr-libibumad"
repos="$repos wfr-mvapich2"
repos="$repos wfr-openmpi"
repos="$repos wfr-gasnet"
repos="$repos wfr-openshmem"
repos="$repos wfr-openshmem-tests"

# skip this, linux kernel 3.12.18 for devel
#repos="$repos wfr-linux-devel"

in_branch=
pull=y
makesrpm=y
diff=n
copy=n
topdir=$PWD
gitdir=GIT_REPOS
verbose=n
while getopts PSdcvb:t:g: param
do
	case $param in
	P)	pull=n;;
	S)	makesrpm=n;;
	d)	diff=y;;
	c)	copy=y;;
	v)	verbose=y;;
	b)	in_branch="$OPTARG";;
	t)	topdir="$OPTARG";;
	g)	gitdir="$OPTARG";;
	*)	Usage;;
	esac
done
shift $((OPTIND -1))
if [ $# -gt 0 ]
then
	repos="$*"
fi

if [ ! -d $topdir ]
then
	echo "gitpull: FAILED: topdir Directory Not Found: $topdir" >&2
	Usage
fi
if [ ! -d $gitdir ]
then
	echo "gitpull: FAILED: gitdir Directory Not Found: $gitdir" >&2
	Usage
fi

# convert topdir to an absolute path
if [ `echo $topdir | cut -c 1` != '/' ]
then
	topdir=$PWD/$topdir
fi
# convert gitdir to an absolute path
if [ `echo $gitdir | cut -c 1` != '/' ]
then
	gitdir=$PWD/$gitdir
fi

[ $verbose = y ] && echo "Using topdir= $topdir; gitdir= $gitdir"


# different repo servers
get_server()
{
	case $1 in
	wfr-lite|wfr-openshmem|wfr-oshmem|wfr-openshmem-tests)	echo $server1;;
	*) echo $server2;;
	esac
}

# some repos have names different from Ofed/ tree name, remap
get_ofed_dir()
{
	case $1 in
	wfr-ibacm)	echo "ibacm";;
	wfr-libibumad)	echo "libibumad";;
	wfr-openshmem-tests)	echo "wfr-openshmem-tests/openshmem-test-suite";;
	*)	echo "$1";;
	esac
}

get_git_branch()
{
	if [ ! -e $topdir/Ofed/$1/commitids ]
	then
		echo "FAILED: Unable to get git branch.  Not Found: $topdir/Ofed/$1/commitids" >&2
	else
		tail -1 $topdir/Ofed/$1/commitids|cut -f 1 -d ' '
	fi
}
 
for i in $repos
do
	branch=$in_branch
	ofed_dir=`get_ofed_dir $i`
	echo "Processing git repo $i against Ofed/$ofed_dir"
	if [ $pull = y ]
	then
		# lookup git branch in commitids file in CVS
		if [ x"$branch" = x ]
		then
			branch=`get_git_branch $ofed_dir`
		fi
		if [ x"$branch" = x ]
		then
			echo "FAILED: unable to determine git branch for $i" >&2
			continue
		fi
		[ $verbose = y ] && echo "Using git branch $branch"

		## get the latest copy of source from the repos
		if [ ! -d $gitdir/$i ]
		then
			server=`get_server $i`
			[ $verbose = y ] && echo "(cd $gitdir; git clone ssh://$USER@$server/$i)"
			(cd $gitdir; git clone ssh://$USER@$server/$i)
		else
			[ $verbose = y ] && echo "(cd $gitdir/$i; git pull)"
			(cd $gitdir/$i; git pull)
		fi
		##Next, checkout the desired branch in each directory
		[ $verbose = y ] && echo "(cd $gitdir/$i; git checkout $branch )"
		(cd $gitdir/$i; git checkout $branch )
	fi
	if [ ! -d $gitdir/$i ]
	then
		echo "FAILED: Need to pull, Directory not found: $gitdir" >&2
		continue
	fi

	##show the branch we are on
	(cd $gitdir/$i; git branch )

	if [ $makesrpm = y ]
	then
		if [ ! -e $gitdir/$i/makesrpm.sh ]
		then
			echo "FAILED: Unable to makesrpm, File not found: $gitdir/$i/makesrpm.sh" >&2
			continue
		fi

			##make the srpm
		[ $verbose = y ] && echo "rm -rf $gitdir/$i/SOURCE_TREE"
		rm -rf $gitdir/$i/SOURCE_TREE
		[ $verbose = y ] && echo "(cd $gitdir/$i/; bash -x ./makesrpm.sh )"
		(cd $gitdir/$i/; bash -x ./makesrpm.sh )

			##expand it
		if [ `ls $gitdir/$i/SRPMS/*.src.rpm|wc -l` != 1 ]
		then
			echo "FAILED: srpm missing or too many, File not found: $gitdir/$i/SRPMS/*.src.rpm" >&2
			continue
		fi
		[ $verbose = y ] && echo "Expanding source into $gitdir/$i/SOURCE_TREE"
		(cd $gitdir/$i/; mkdir SOURCE_TREE; $topdir/MakeTools/expand_source.sh -d SOURCE_TREE SRPMS/*.src.rpm)
	fi

	if [ $diff = y ]
	then
		if [ ! -e $gitdir/$i/SOURCE_TREE/package_list ]
		then
			echo "FAILED: Unable to diff, File not found: $gitdir/$i/SOURCE_TREE/package_list" >&2
			continue
		fi

			## use package_list because srpm name is not necessarily $i
		package=`head -1 $gitdir/$i/SOURCE_TREE/package_list`
		[ $verbose = y ] && echo "Diff $gitdir/$i/SOURCE_TREE/$package/ $topdir/Ofed/$ofed_dir/ ..."
		diff -b -u --no-dereference --exclude CVS --exclude .git --exclude package_list -r $gitdir/$i/SOURCE_TREE/$package/ $topdir/Ofed/$ofed_dir/
	fi

	if [ $copy = y ]
	then
		if [ ! -e $gitdir/$i/SOURCE_TREE/package_list ]
		then
			echo "FAILED: Unable to copy, File not found: $gitdir/$i/SOURCE_TREE/package_list" >&2
			continue
		fi

		[ $verbose = y ] && echo "Replacing source in $topdir/Ofed/$i ..."
		##copy expanded srpm source from SOURCE_TREE to CVS Ofed/ tree
			##remove existing files in Ofed/ checkout
		find $topdir/Ofed/$ofed_dir/src/ ! -type d| grep -v CVS|xargs rm -f
			##replace with new files from srpm
			## use package_list because srpm name is not necessarily $i
		package=`head -1 $gitdir/$i/SOURCE_TREE/package_list`
		# use find to avoid symlinks, make_symlinks will handle them when build
		#cp -r $gitdir/$i/SOURCE_TREE/$package/* $topdir/Ofed/$ofed_dir/
		(cd $gitdir/$i/SOURCE_TREE/$package; find . -type d -o -type f|cpio -pdvum $topdir/Ofed/$ofed_dir/)

	fi
done 2>&1|tee -a gitpull.log
