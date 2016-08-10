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
# $Source: /cvs/ics/MakeTools/Test/Err.msg,v $
# $Name:  $
# $Revision: 1.3 $
# $Date: 2015/01/28 20:18:11 $
#

# 
# Message Definitions for Err module
#

[Module]
Name=Err

[String]
Name=OK
Comment=ERR_OK text
String.English="Success"

[String]
Name=FAILED_CONSTRUCTOR
Comment=ERR_FAILED_CONSTRUCTOR and ERR_KIND_FAILED_CONSTRUCTORtext
String.English="Failed Constructor"

[String]
Name=END_OF_DATA
Comment=ERR_END_OF_DATA and ERR_KIND_END_OF_DATA text
String.English="End of Data"

[String]
Name=IO_ERROR
Comment=ERR_IO_ERROR and ERR_KIND_IO_ERROR text
String.English="IO Error"

[String]
Name=INVALID
Comment=ERR_INVALID and ERR_KIND_INVALID text
String.English="Invalid"

[String]
Name=MISSING
Comment=ERR_MISSING and ERR_KIND_MISSING text
String.English="Missing"

[String]
Name=DUPLICATE
Comment=ERR_DUPLICATE and ERR_KIND_DUPLICATE text
String.English="Duplicate"

[String]
Name=OVERFLOW
Comment=ERR_OVERFLOW and ERR_KIND_OVERFLOW text
String.English="Overflow"

[String]
Name=TIMEOUT
Comment=ERR_TIMEOUT and ERR_KIND_TIMEOUT text
String.English="Timeout"

[String]
Name=UNDERFLOW
Comment=ERR_UNDERFLOW and ERR_KIND_UNDERFLOW text
String.English="Underflow"

[String]
Name=OUT_OF_RESOURCES
Comment=ERR_OUT_OF_RESOURCES and ERR_KIND_OUT_OF_RESOURCES text
String.English="Out of Resources"

[String]
Name=UNKNOWN
Comment=ERR_KIND_UNKNOWN text
String.English="Unknown"

[String]
Name=BAD
Comment=invalid Err_Code_t value for Err module defined errors
String.English="Bad"
