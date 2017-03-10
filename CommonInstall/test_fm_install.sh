#!/bin/bash
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

#[ICS VERSION STRING: unknown]

# this script allows simple unit testing of IntelOPA-FM installer.  To run
# this, first install appropriate version of OFED, then cd into IntelOPA-FM-*/
# directory (either subtree of IntelOPA-IFS or a direct build of VIEO_HOST)
# and run this script.  If any of the tests fail, they say FAIL.
# also review the output carefully (recommend using "script" command when
# running)
# This requires a build of VIEO_HOST from HEAD or 4.4 after 9pm 11/5/2008
# coding being tested was checked in at that time

rm_config()
{
	rm -f /etc/sysconfig/iview_fm.config /etc/sysconfig/*rpmnew /etc/opa-fm/opafm.xml
}

check_default()
{
	diff /etc/opa-fm/opafm.xml-sample /etc/opa-fm/opafm.xml
	[ $? != 0 ] && echo "FAIL: installed does not match sample"
	diff /etc/opa-fm/opafm.xml-sample /usr/share/opa-fm/opafm.xml
	[ $? != 0 ] && echo "FAIL: sample does not match default"
}

check_nodefault()
{
	diff /etc/opa-fm/opafm.xml-sample /etc/opa-fm/opafm.xml
	[ $? != 0 ] || echo "FAIL: installed should not match sample"
	diff /etc/opa-fm/opafm.xml-sample /usr/share/opa-fm/opafm.xml
	[ $? != 0 ] && echo "FAIL: sample does not match default"
}


check_noiview()
{
	[ -f /etc/sysconfig/iview_fm.config ] && echo "FAIL: iview_fm.config installed"
}

check_iview()
{
	[ -f /etc/sysconfig/iview_fm.config ] || echo "FAIL: iview_fm.config lost"
}

check_norpmnew()
{
	[ -f /etc/sysconfig/*rpmnew ] && echo "FAIL: rpmnew found"
}


set -x
echo "========================================================================="
echo "Install with no existing config"
rm_config
./INSTALL -a
check_default
check_noiview
check_norpmnew

echo "========================================================================="
echo "Install with iview_fm.config matching 4.4 version"
rm_config
cp /usr/lib/opa-fm/etc/iview_fm.config.4x /etc/sysconfig/iview_fm.config
./INSTALL -a
check_default
check_iview
check_norpmnew

echo "========================================================================="
echo "Install with iview_fm.config matching 4.3 version"
rm_config
cp /usr/lib/opa-fm/etc/iview_fm.config.4.3 /etc/sysconfig/iview_fm.config
./INSTALL -a
check_default
check_iview
check_norpmnew

echo "========================================================================="
echo "Install with iview_fm.config matching 4.3.1 version"
rm_config
cp /usr/lib/opa-fm/etc/iview_fm.config.4.3.1 /etc/sysconfig/iview_fm.config
./INSTALL -a
check_default
check_iview
check_norpmnew

echo "========================================================================="
echo "Install with iview_fm.config matching 4.2.1.2.1 version"
rm_config
cp /usr/lib/opa-fm/etc/iview_fm.config.4.2.1.2.1 /etc/sysconfig/iview_fm.config
./INSTALL -a
check_default
check_iview
check_norpmnew

echo "========================================================================="
echo "Install with opafm.xml matching 4.4 version"
rm_config
cp /usr/share/opa-fm/opafm.xml /etc/opa-fm/opafm.xml
./INSTALL -a
check_default
check_noiview
check_norpmnew

echo "========================================================================="
echo "Install with opafm.xml matching sample but not 4.4 version (eg. upgrade)"
rm_config
cp /usr/share/opa-fm/opafm.xml /etc/opa-fm/opafm.xml
echo "<!-- extra line -->" >> /etc/opa-fm/opafm.xml
cp /etc/opa-fm/opafm.xml /etc/opa-fm/opafm.xml-sample
./INSTALL -a
check_default
check_noiview
check_norpmnew

echo "========================================================================="
echo "Install with modified opafm.xml"
rm_config
cp /usr/share/opa-fm/opafm.xml /etc/opa-fm/opafm.xml
echo "<!-- extra line -->" >> /etc/opa-fm/opafm.xml
./INSTALL -a
check_nodefault
check_noiview
check_norpmnew

echo "========================================================================="
echo "Install with modified iview_fm.config"
rm_config
cp /usr/lib/opa-fm/etc/iview_fm.config.4x /etc/sysconfig/iview_fm.config
echo "#extra line" >> /etc/sysconfig/iview_fm.config
./INSTALL -a
check_nodefault
check_iview
check_norpmnew

