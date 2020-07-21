#!/bin/bash

###################################################################################
#
# unifdef2.sh script
#
# Arguments:
#   in-file - the input file. The file must contain at least one instance of
#     "ifdef <flag>" and "endif <flag>" bracketing some text
#   out-file - the output file to be written after applying the filtering
#
#   unifdef2.sh filters a file by including everything outside of ifdef-endif
#     brackets, and including only lines inside ifdef-endif brackets comparing
#     the flag settings to environment settings. All other lines are filtered out.
#     Also support ifndef as well as "ifdef !"
#
#   Syntax of ifdef-endif pairs:
#     Bracketing lines is accomplished by using "#ifdef <flag>" and "#endif"
#     lines starting in the first colunm. There must be a single space between
#     the ifdef keyword and the flag. Else and endif statements to not have the flag
#
#   Calling example: MakeTools/unifdef2.sh foo.sh.base foo.sh
#     This example calls the script on the input file "foo.sh.base", processing
#     the file looking for ifdef-endif bracketing pairs, and excluding all code
#     within those brackets except those bracketed by "#ifdef <flag>".."#endif"
#     that match environment settings.
#
###################################################################################

#
# Functions for handling ifdefs-by-export-flag
# for buildFeatureDefs
#

get_flags()
{
	local line="$1"
	local flags=$(echo $line | cut -d' ' -f2-)
	echo "$flags"
}

evaluate_flags ()
{
	local flags_good=1
	local flags="$1"
	local op=
	for flag in $flags
	do
		if [ "$flag" = "AND" ]
		then
			op="and"
			continue
		elif [ "$flag" = "OR" ]
		then
			op="or"
			continue
		else
			if ! echo $flag | grep "=" > /dev/null
			then
				echo syntax error
				exit 1
			fi
		fi
		f=$(echo $flag | cut -d= -f1)
		flagval=$(echo $flag | cut -d= -f2)
		flag_env_val=${!f}
		if [ "$flag_env_val" = "" ]
		then
			# flag not set in env, assume zero
			flag_env_val=0
		fi
		if [ "$flagval" = "$flag_env_val" ]
		then
			this_flag_good=1
		else
			this_flag_good=0
		fi
		if [ "$op" = "or" ]
		then
			flags_good=$(($flags_good | $this_flag_good))
		elif [ "$op" = "and" ]
		then
			flags_good=$(($flags_good & $this_flag_good))
		else
			# first flag will not have "op" set, so just set it to this_glag_good
			flags_good=$this_flag_good
		fi
	done
	echo $flags_good
}
setup_env()
{
	# Import OPA Build feature settings.
	export OPA_FEATURE_SET=${OPA_FEATURE_SET:-$(cat $TL_DIR/$PROJ_FILE_DIR/DEFAULT_OPA_FEATURE_SET)}
	FEATURE_SETTINGS_FILE=opa_feature_settings.${PRODUCT}.$BUILD_CONFIG
	$TL_DIR/OpaBuildFeatureToggles/opa_build_import_feature_settings.sh
	if [ -e ./$FEATURE_SETTINGS_FILE ]; then
		. ./$FEATURE_SETTINGS_FILE
	else
		echo "$0: ERROR: OPA BUILD - $FEATURE_SETTINGS_FILE not found" >&2
		return 1
	fi
}

check_flag_dependencies()
{

	# if A21 or APR_ASIC are set, APR must be set

	if [ "$OPA_FEATURE_ENABLE_FM_A21" = "1" -a "$OPA_FEATURE_ENABLE_APR" = "0" ]
	then
		echo "Error: OPA Feature Settings: OPA_FEATURE_ENABLE_FM_A21 cannot be 1 unless OPA_FEATURE_ENABLE_APR is 1"
		exit 1
	fi
	if [ "$OPA_FEATURE_ENABLE_FM_APR_ASIC" = "1" -a "$OPA_FEATURE_ENABLE_APR" = "0" ]
	then
		echo "Error: OPA Feature Settings: OPA_FEATURE_ENABLE_FM_APR_ASIC cannot be 1 unless OPA_FEATURE_ENABLE_APR is 1"
		exit 1
	fi
}

#
# Functions for handling ifdefs-by-feature
# for all files other than buildFeatureDefs
#

declare -a defines
declare -a undefs
declare -a ifdef_stack
stack_size=0

evaluate_flag_nofd()
{
	not=0

	flag="$1"
	if [[ $flag =~ ^! ]]
	then
		not=1
		flag=${flag:1}
	fi

	flag_defined=-1
	for i in ${defines[@]}
	do
		if [ "$i" = "$flag" ]
		then
			flag_defined=1
			break
		fi
	done
	if [ $flag_defined -lt 0 ]
	then
		for i in ${undefs[@]}
		do
			if [ "$i" = "$flag" ]
			then
				flag_defined=0
				break
			fi
		done
	fi
	if [ $flag_defined -lt 0 ]
	then
		echo ERROR: invalid flag $flag
		exit 1
	fi

	if [ $not -eq 1 ]
	then
		flag_defined=$((1 - flag_defined))
	fi

	echo $flag_defined

}

push_flag()
{
	ifdef_stack[$stack_size]=$1
	stack_size=$((stack_size + 1))
}

pop_flag()
{
	stack_size=$((stack_size - 1))
	unset ifdef_stack[$stack_size]
}

top_of_stack()
{
	last_index=$((stack_size - 1))
	if [ $last_index -ge 0 ]
	then
		echo ${ifdef_stack[$last_index]}
	else
		echo ""
	fi
}

get_flag_nofd()
{
	echo "$line" | cut -d' ' -f 2-
}

evaluate_flag()
{
	if [ $featureDefs -eq 1 ]
	then
		flag_defined=$(evaluate_flags "$2")
	else
		flag_defined=$(evaluate_flag_nofd "$2")
	fi

	if [ $1 = ifndef ]
	then
		flag_defined=$((1 - flag_defined))
	fi
	echo $flag_defined
}

get_flag()
{
	if [ $featureDefs -eq 1 ]
	then
		echo $(get_flags "$1")
	else
		echo $(get_flag_nofd "$1")
	fi
}

# shut off globbing
set -f

if [ $# -ne 2 ]
then
	echo ERROR: unifdef2.sh: wrong number of arguments: $#
	echo Usage: unifdef2.sh in-file out-file
	exit 1
fi

in_file=$1
out_file=$2

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
	echo Usage: unifdef2.sh in-file out-file
	exit 1
fi

# if out_file is buildFeatureDefs, set featureDefs

if echo $out_file | grep buildFeatureDefs > /dev/null
then
	featureDefs=1
else
	featureDefs=0
fi


# parse for syntax - check ifdef and endif tags, ensure matching and no nesting

# first extract all #ifdef and #endif directives to make it run faster
# ... in the grep, record the line nu㏔ers as well, we will use in main loop below

grep -n "^#ifdef .*" $in_file > .greps1
grep -n "^#ifndef .*" $in_file >> .greps1
grep -n "^#else" $in_file >> .greps1
grep -n "^#endif" $in_file >> .greps1
sort -g .greps1 > .greps # sort using line numbers with general-number-sort
rm -f .greps1

# check for:
#  - mismatched ifdef-endif pairs
#  - mismatched else
#  - duplicate else
#  - else before ifdef
#  - endif before ifdef
#  - nesting

inside_bracket=0
in_else=0
while read line
do
	if [ $inside_bracket -eq 0 ]
	then
		if echo $line | grep "#ifdef " > /dev/null
		then
			inside_bracket=$((inside_bracket+1))
			in_else=0
		elif echo $line | grep "#ifndef " > /dev/null
		then
			inside_bracket=$((inside_bracket+1))
			in_else=0
		elif echo $line | grep "#else" > /dev/null
		then
			echo ERROR: unifdef2.sh: else before ifdef/ifndef in $in_file
		elif echo $line | grep "#endif" > /dev/null
		then
			echo ERROR: unifdef2.sh: endif before ifdef/ifndef in $in_file
		fi
	else # inside bracket - look for endif for this tag
		if echo $line | grep "#ifdef " > /dev/null
		then
			inside_bracket=$((inside_bracket+1))
			in_else=0
		elif echo $line | grep "#ifndef " > /dev/null
		then
			inside_bracket=$((inside_bracket+1))
			in_else=0
		elif echo $line | grep "#endif" > /dev/null
		then
			inside_bracket=$((inside_bracket-1))
			in_else=0
		elif echo $line | grep "#else" > /dev/null
		then
			if [ "$in_else" = "1" ]
			then
				echo ERROR: unifdef2.sh: duplicate else in $in_file
				exit 1
			fi
			in_else=1
		fi
	fi
done < .greps
if [ $inside_bracket -gt 0 ]
then
	echo ERROR: unifdef2.sh: no endif for ifdef/ifndef in $in_file
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
#     grab flag and directive from line
#     if directive is ifdef
#       if flag is in defines
#         set writing to 1
#		elif flag is in undefs
#         set writing to 0
#		else
#         exit bad flag
#     elif directive is else
#     else (directive is endif)
#       if flags do not match
#         set writing to 1 (start writing again, "filter out" bracket is closed)
#   else (not a directive, just a normal line)
#     if writing is 1, write line to out_file
#   increment line number
#

> $out_file
chmod --reference=$in_file $out_file		# match permissions of in-file

# if doing featureDefs
#   check flag dependencies and environment
# else
#   load arrays

if [ $featureDefs -eq 1 ]
then
	if [ "$FLAGS_INITIALIZED" != "yes" ]
	then
		setup_env || exit 1
	fi
	check_flag_dependencies
else
	define_index=0
	undef_index=0
	while read line
	do
		if echo "$line" | grep "#define" > /dev/null
		then
			var=$(echo "$line"  | cut -f2 -d# | cut -f2 -d' ')
			defines[$define_index]=$var
			define_index=$((define_index+1))
		fi
		if echo "$line" | grep "#undef" > /dev/null
		then
			var=$(echo "$line" | cut -f2 -d# | cut -f2 -d' ')
			undefs[$undef_index]=$var
			undef_index=$((undef_index+1))
		fi
	done < $TL_DIR/Fd/buildFeatureDefs
fi

writing=1
line_number=1

while IFS= read -r line
do
	# is line_number in .greps?
	if grep -w ^$line_number .greps > /dev/null
	then
		flag=$(get_flag "$line")
		ldirective=$(grep -w ^$line_number .greps | cut -d' ' -f1 | cut -d# -f2)
		if [ "$ldirective" = "ifdef" -o "$ldirective" = "ifndef" ]
		then
			save_directive=$ldirective
			push_flag "$flag"
			flag_defined=$(evaluate_flag $ldirective "$flag")
			if [ "$flag_defined" = "1" ]
			then
				writing=1
			else
				writing=0
			fi
		elif [ "$ldirective" = "else" ]
		then
			top="$(top_of_stack)"
			flag_defined=$(evaluate_flag $ldirective "$top")
			if [ "$flag_defined" = "0" ]
			then
				writing=1
			else
				writing=0
			fi
		else # it is endif
			pop_flag
			save_directive=
			writing=1
		fi
	elif [ $writing -eq 1 ]
	then
		echo "$line" >> $out_file
	fi
	line_number=$((line_number+1))
done < $in_file

rm -f .greps

exit 0
