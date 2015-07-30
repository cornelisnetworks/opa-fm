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
# Windows Visual Studio DSP stuff
# this is part of the make system ability to create dsp/dsw
# files for Microsoft Visual Studio, hence make is primary tool
# but users who want to use Visual Studio can still use it for edit
# and build, but not so much for execute and debug

DS_PROJECT		= $(notdir $(CURDIR))
DSP_SUFFIX		= _$(BUILD_TARGET).dsp
DSW_SUFFIX		= _$(BUILD_TARGET).dsw
DSP_FILE		= $(DS_PROJECT)$(DSP_SUFFIX)
DSW_FILE		= $(DS_PROJECT)$(DSW_SUFFIX)
DSP_TMP_FILE	= $(DS_PROJECT)_tmp$(DSP_SUFFIX)
DSW_TMP_FILE	= $(DS_PROJECT)_tmp$(DSW_SUFFIX)
DS_PROJECTS		= Makerules MakeTemplates CodeTemplates
DS_PROJECT_DIR	= $(CURDIR)
DSP_SRC_DIR		= .$(DSP_GROUP)
