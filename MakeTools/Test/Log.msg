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
# $Source: /cvs/ics/MakeTools/Test/Log.msg,v $
# $Name:  $
# $Revision: 1.3 $
# $Date: 2015/01/28 20:18:11 $
#

# 
# Message Definitions for Log module
#

# WHEN TRANSLATING, ALSO UPDATE DUMMY MESSAGES IN Log_Subsystem.cpp

[Module]
Name=Log

[Message]
Name=TRUNCATED
Severity=Warning
Description.English="Message truncated, out of Log Resources"
Response.English="Some log text has been discarded"
Correction.English="To avoid the loss of log text, reduce the logging options enabled"

[Message]
Name=TRAP_FAILED
Severity=Warning
Arglist=erc
Description.English="Unable to Send Trap: %C"

[String]
Name=RESPONSE
Comment=Prefix for Response part of message
String.English="Response: "

[String]
Name=CORRECTION
Comment=Prefix for Correction part of message
String.English="Correction: "

[String]
Name=TASK
Comment=Task Id reporting message
String.English="Task"

[String]
Name=FILENAME
Comment=Source file generating message
String.English="Filename"

[String]
Name=LINE
Comment=Source file line number generating message
String.English="Line"

[String]
Name=NULL
Comment=marker to indicate a NULL %s or %F argument was passed
String.English="(NULL)"

[String]
Name=NOMEMORY
Comment=marker to indicate a LOG_ALLOC_ERROR argument was passed
String.English="(No Memory)"

[String]
Name=SEV_DUMP
Comment=1 character indication of severity
String.English="D"

[String]
Name=SEV_FATAL
Comment=1 character indication of severity
String.English="F"

[String]
Name=SEV_ERROR
Comment=1 character indication of severity
String.English="E"

[String]
Name=SEV_ALARM
Comment=1 character indication of severity
String.English="A"

[String]
Name=SEV_WARNING
Comment=1 character indication of severity
String.English="W"

[String]
Name=SEV_PARTIAL
Comment=1 character indication of severity
String.English="p"

[String]
Name=SEV_CONFIG
Comment=1 character indication of severity
String.English="C"

[String]
Name=SEV_PROGRAM_INFO
Comment=1 character indication of severity
String.English="I"

[String]
Name=SEV_PERIODIC_INFO
Comment=1 character indication of severity
String.English="P"

[String]
Name=SEV_DEBUG1
Comment=1 character indication of severity
String.English="1"

[String]
Name=SEV_DEBUG2
Comment=1 character indication of severity
String.English="2"

[String]
Name=SEV_DEBUG3
Comment=1 character indication of severity
String.English="3"

[String]
Name=SEV_DEBUG4
Comment=1 character indication of severity
String.English="4"

[String]
Name=SEV_DEBUG5
Comment=1 character indication of severity
String.English="5"

[String]
Name=SEV_PURGE
Comment=1 character indication of severity
String.English="X"
