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
#
# $Source: /cvs/ics/MakeTools/Test/Test.msg,v $
# $Name:  $
# $Revision: 1.4 $
# $Date: 2015/01/28 20:18:11 $
#

# 
# Message Definitions for Osa module
#

[Module]
Name=Osa

[Message]
Name=ASSERT_FAILED
Severity=Fatal
Arglist=expr: text for failed expression
Description.English="Assertion Failed: %s"
Response.English="Firmware will dump and reboot"
Correction.English="Save the Dump and send it to support"

[Message]
Name=TEST1
Severity=Fatal
Arglist=obj1
Description.English="%F"	# strip this
[Message]
Name=TEST2
Severity=Fatal
Arglist=arg1, arg2, obj1, obj2
Description.English="%3$F %4$F"
[Message]
Name=TEST3
Severity=Fatal
Arglist=obj1, obj2, width1, width2
Description.English="%1$*3$F %2$*4$F"
[Message]
Name=TEST4
Severity=Fatal
Arglist=obj1, obj2, width1, width2
Description.English="%1$.*3$F %2$.*4$F"
[Message]
Name=TEST5
Severity=Fatal
Arglist=width1, obj1, width2, obj2
Description.English="%*F %*F"
[Message]
Name=TEST6
Severity=Fatal
Arglist=width1, obj1, width2, obj2
Description.English="%.*F %.*F"
[Message]
Name=TEST7
Severity=Fatal
Arglist=width1a, width1b, obj1, width2a, width2b, obj2
Description.English="%*.*F %*.*F"
[Message]
Name=TEST8
Severity=Fatal
Arglist=dec, obj2, width1, width2
Description.English="%*3$2$F %*4$1$d"

[Message]
Name=TEST9
Severity=Partial
Description.English="#"
[Message]
Name=TEST10
Severity=Partial
Description.English="\"#"
[Message]
Name=TEST11
Severity=Partial
Description.English="#\""
[Message]
Name=TEST12
Severity=Partial
Description.English="\" #" # strip
