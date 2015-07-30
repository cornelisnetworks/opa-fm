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
# Makefile for Makerules, just to create a dsp file

# Include Make Control Settings
include $(TL_DIR)/$(PROJ_FILE_DIR)/Makesettings.project

DS_SUBPROJECTS = 
DSP_SOURCES= Defs.BuildPlatform.CYGWIN \
			Defs.BuildPlatform.DOS \
			Defs.BuildPlatform.LINUX \
			Defs.BuildPlatform.DARWIN \
			Defs.Common \
			Defs.Dsp \
			Defs.Flags \
			Defs.GNU \
			Defs.Paths \
			Defs.Targets \
			Defs.Tornado \
			Defs.Wrs \
			Makefile \
			Makerules.global \
			Maketargets.bsp \
			Maketargets.build \
			Maketargets.install \
			Maketargets.moduleinstall \
			Maketargets.modulestage \
			Maketargets.runtest \
			Maketargets.stage \
			Maketargets.tms \
			Maketargets.toplevel \
			README \
			Rules.Common \
			Rules.CYGWIN \
			Rules.DOS \
			Rules.LINUX \
			Rules.DARWIN \
			Target.CYGWIN.GNU.CYGWIN \
			Target.LINUX.GNU.EM64T \
			Target.LINUX.GNU.IA32 \
			Target.LINUX.GNU.IA64 \
			Target.LINUX.GNU.MIPS64 \
			Target.LINUX.GNU.X86_64 \
			Target.LINUX.INTEL.IA32 \
			Target.LINUX.INTEL.IA64 \
			Target.DARWIN.GNU.PPC \
			Target.VXWORKS.GNU.ATOM \
			Target.VXWORKS.GNU.I386 \
			Target.VXWORKS.GNU.MIPS \
			Target.VXWORKS.GNU.MIPS64 \
			Target.WIN32.WIN32.WIN32 \
			template.dsp

DIRS		= 

INCLUDE_TARGETS		= 

include $(TL_DIR)/$(PROJ_FILE_DIR)/Makerules.project

include $(TL_DIR)/Makerules/Maketargets.toplevel

INSTALL::

STAGE::
