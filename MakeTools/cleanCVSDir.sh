#!/bin/bash

if [ "$1" = "" ]
then
	echo Usage: cleanCVSDir.sh dirname
	exit 1
fi

if [ -d "$1" ]
then
	for i in $1/*
	do
		if [ -f "$1/$i" ]
		then
			# it is a file, just remove it
			rm -f $1/$i
		elif [ "$i" != "CVS" ]
		then
			# it is a directory, not the CVS directory
			# if it does not have a CVS directory in it, remove it
			# otherwise, it is a repository directory, so recurse
#			LS=`ls -d $1/$i/CVS 2>/dev/null`
#			if [ "$LS" == "" ]

			if [ ! -d "$1/$i/CVS" ]
			then
				rm -rf $1/$i
			else
				$TL_DIR/MakeTools/cleanCVSDir.sh $1/$i
			fi
		fi
	done

	if [ \( ! -d "$1/CVS" \) -a "$1" != "." ]
	then
		rm -rf "$1"
	fi

fi
