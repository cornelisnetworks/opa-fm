#! /bin/bash
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

failed=0
tests=0
log=test.$$
cmd="perl ../buildmsgs.pl"

error() {
	echo $*
	[ -s $log ] && cat $log
	let failed+=1
}

test_file() {
	let tests+=1
	out=`basename $1 .msg`_Messages
	if $cmd $1 > $log 2>&1 ; then
		if [ -s $log ] ; then
			error "Unexpected output from buildmsgs.pl $1"
			return 1
		else
			for file in $out.*; do
				echo "diff expect/$file $file" > $log
				diff expect/$file $file >> $log
				if [ $? != 0 ]; then
					error "$file"
					return 1
				fi
			done
		fi
	else
		error "buildmsgs.pl $1 FAILED"
		return 1
	fi
	rm $out.*
	return 0
}

# regression tests
for msg in *.msg; do
	echo -n "$msg: "
	if test_file $msg; then
		echo "PASSED"
	else
		echo "FAILED"
	fi
done

# check error output
echo -n "Errors: "
let tests+=1
if $cmd 'ErrTest.in' > $log 2> 'ErrTest.err'; then
	error "No error returned"
	echo "FAILED"
elif [ -s $log ]; then
	error "Unexpected output to stdout"
	echo "FAILED"
elif [ -f "ErrTest_Messages.c" ] || [ -f "ErrTest_Messages.h" ]; then
	error "Output files created"
	echo "FAILED"
else
	echo "diff expect/ErrTest.err ErrTest.err" > $log
	diff expect/ErrTest.err ErrTest.err >> $log
	if [ $? != 0 ]; then
		error "ErrTest.err"
		echo "FAILED"
	else
		echo "PASSED"
		rm ErrTest.err
	fi
fi

# check format strings
echo -n "Formats: "
let tests+=1
if $cmd --test < Formats.in > Formats.out 2> $log ; then
	if [ -s $log ]; then
		error "Unexpect output to stderr"
		echo "FAILED"
	else
		echo "diff expect/Formats.out Formats.out" > $log
		diff expect/Formats.out Formats.out >> $log
		if [ $? != 0 ]; then
			error "Formats.out"
			echo "FAILED"
		else
			echo "PASSED"
			rm Formats.out
		fi
	fi
else
	error "Error returned"
	echo "FAILED"
fi

let passed=tests-failed
echo "$tests Tests Run, $passed Passed, $failed Failed"

rm $log

if [ $failed != 0 ]
then
	exit 1
else
	exit 0
fi

