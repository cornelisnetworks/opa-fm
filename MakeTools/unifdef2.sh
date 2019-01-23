#!/bin/bash

###################################################################################
#
# unifdef2.sh script
#
# Arguments:
#   featureset-tag - this names the feature set and is used to identify sections
#     in the in-file to be considered in the out-file
#   in-file - the input file. The file must contain at least one instance of
#     "ifdef <tag>" and "endif <tag>" bracketing some text
#   out-file - the output file to be written after applying the filtering
#
#   unifdef2.sh filters a file by including everything outside of ifdef-endif
#     brackets, and including only lines inside ifdef-endif brackets with the
#     given featureset tag. All other lines are filtered out.
#
#   Syntax of ifdef-endif pairs:
#     Bracketing lines is accomplished by using "#ifdef <tag>" and "#endif <tag>"
#     lines starting in the first colunm. There must be a single space between
#     the ifdef/endif keyword and the tag. The tag can be followed by whitespace
#     and then optional commenting text.
#     Tags can be made up of alphnumeric and underscore characters, for example,
#     "opa10", "opa11_2", etc.
#     Examples:
#     #ifdef opa10
#     <some lines of text
#     #endif opa10 # this can be comments, the pound sign is not necessary
#
#   Calling example: MakeTools/unifdef2.sh opa10 foo.sh.base foo.sh
#     This example calls the script on the input file "foo.sh.base", processing
#     the file looking for ifdef-endif bracketing pairs, and excluding all code
#     within those brackets except those bracketed by "#ifdef opa10".."#endif opa10"
#
###################################################################################

check_tag_syntax()
{
	ltag1=$1
	where="$2"

	rest=${ltag1//[A-Za-z0-9_]}
	if [ "$rest" != "" ]
	then
		echo ERROR: unifdef2.sh: invalid featureset \<$ltag1\> "$where"
		echo        featureset can only be alphanumeric and underscores
		rm -f .greps .greps1
		exit 1
	fi
}

# shut off globbing
set -f

if [ $# -ne 3 ]
then
	echo ERROR: unifdef2.sh: wrong number of arguments: $#
	echo Usage: unifdef2.sh featureset-tag in-file out-file
	exit 1
fi

featureset=$1
in_file=$2
out_file=$3

check_tag_syntax $featureset "on command line"

# check existence of in_file

if [ ! -f $in_file ]
then
	echo ERROR: unifdef2.sh: no such file $in_file
	exit 1
fi

# check that in_file and out_file are different

if [ "$in_file" = "$out_file" ]
then
	echo ERROR: unifdef2.sh: in-file must be different than out-file
	echo Usage: unifdef2.sh featureset-tag in-file out-file
	exit 1
fi

# parse for syntax - check ifdef and endif tags, ensure matching and no nesting

# first extract all #ifdef and #endif directives to make it run faster
# ... in the grep, record the line nu㏔ers as well, we will use in main loop below

grep -n "^#ifdef .*" $in_file > .greps1
grep -n "^#endif .*" $in_file >> .greps1
sort -g .greps1 > .greps # sort using line numbers with general-number-sort
rm -f .greps1

# check for:
#  - mismatched ifdef-endif pairs
#  - endif before ifdef
#  - nesting

outside_bracket=1
save_tag=
while read line
do
	if [ $outside_bracket = 1 ]
	then
		if echo $line | grep "#ifdef " > /dev/null
		then
			save_tag=$(echo $line | cut -d' ' -f 2 | cut -f1)
			check_tag_syntax $save_tag "in $in_file"
			outside_bracket=0
		elif echo $line | grep "#endif " > /dev/null
		then
			save_tag=$(echo $line | cut -d' ' -f 2 | cut -f1)
			check_tag_syntax $save_tag "in $in_file"
			echo ERROR: unifdef2.sh: endif tag before ifdef for $save_tag in $in_file
			exit 1
		fi
	else # inside bracket - look for endif for this tag
		if echo $line | grep "#endif $save_tag" > /dev/null
		then
			save_tag=
			outside_bracket=1
		elif echo $line | grep "#ifdef " > /dev/null
		then
			save_tag2=$(echo $line | cut -d' ' -f 2 | cut -f1)
			check_tag_syntax $save_tag2 "in $in_file"
			echo ERROR: unifdef2.sh: ifdef tag for $save_tag2 before endif for $save_tag in $in_file
			exit 1
		elif echo $line | grep "#endif " > /dev/null
		then
			save_tag2=$(echo $line | cut -d' ' -f 2 | cut -f1)
			check_tag_syntax $save_tag2 "in $in_file"
			echo ERROR: unifdef2.sh: endif tag for $save_tag2 inside bracket for $save_tag in $in_file
			exit 1
		fi
	fi
done < .greps
if [ $outside_bracket = 0 ]
then
	echo ERROR: unifdef2.sh: no endif for ifdef for $save_tag in $in_file
	exit 1
fi


#
# now filter the file to create the out file
#
# Set writing to 1 (write everything except what is inside other ifdefs)
# Set line number to 1
#
# Loop through lines in in_file:
#   read a line
#   if the line number matches an ifdef/endif directive
#     grab tag and directive from line
#     if directive is ifdef
#       if tag does not match featureset
#         set writing to 0 (filter out this stuff)
#     else (directive is endif)
#       if tag does not match featureset
#         set writing to 1 (start writing again, "filter out" bracket is closed)
#   else (not a directive, just a normal line)
#     if writing is 1, write line to out_file
#   increment line number
#

> $out_file
chmod --reference=$in_file $out_file		# match permissions of in-file

writing=1
line_number=1

while IFS= read -r line
do
	# is line_number in .greps?
	if grep -w $line_number .greps > /dev/null
	then
		ltag=$(grep -w $line_number .greps|cut -f2 -d' '|cut -f1)
		ldirective=$(grep -w $line_number .greps | cut -d' ' -f1 | cut -d# -f2)
		if [ "$ldirective" = "ifdef" ]
		then
			if [ "$ltag" != "$featureset" ]
			then
				writing=0
			fi
		else # it is endif
			if [ "$ltag" != "$featureset" ]
			then
				writing=1
			fi
		fi
	elif [ $writing -eq 1 ]
	then
		echo "$line" >> $out_file
	fi
	line_number=$((line_number+1))
done < $in_file

rm -f .greps

exit 0
