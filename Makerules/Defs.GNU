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
# included by Target.* files, typically only used by those for
# BUILD_TARGET_TOOLCHAIN=GNU
# flags for building with GNU compilers

# C/C++ compilation
# ----------------------------
CANSI		= -ansi
CPIC		= -fPIC
CPIE		= 
CCPIC		= $(CPIC)
CCEXH		= -fexceptions
CCNOEXH		= -fno-exceptions
CCRTTI		= -frtti
CCNORTTI	= -fno-rtti
CSHARED		= 
CCSHARED	= 
CINCSRCH	= $(foreach inc,$(CINCLUDE_DIRS),-I$(inc))

# linking
# ----------------------------
LDSTRIP		=
LDOUTPUT	= -o$(space)
LDLIBSRCH	= $(foreach lib,$(LD_LIB_DIRS),-L$(lib))
LDSTATIC	= -Xlinker -non_shared

LEX_LIB		= fl

# to assist module makefiles, create ARLIBS listing libraries to include
# in build and be combined into one module level library
ifeq "$(strip $(LIBFILES))" ""
ARLIBS=
else
ARLIBS=$(foreach lib, $(notdir $(LIBFILES)), $(MOD_LIB_DIR)/$(notdir $(LIB_PREFIX)$(lib)$(ARLIB_SUFFIX)))
endif

__GNUC__		= $(shell touch tmp_dME.c;gcc -dM -E tmp_dME.c|grep -w __GNUC__|cut -d" " -f3;rm -f tmp_dME.c)
__GNUC_MINOR__		= $(shell touch tmp_dME.c;gcc -dM -E tmp_dME.c|grep -w __GNUC_MINOR__|cut -d" " -f3;rm -f tmp_dME.c)
__GNUC_PATCHLEVEL__	= $(shell touch tmp_dME.c;gcc -dM -E tmp_dME.c|grep -w __GNUC_PATCHLEVEL__|cut -d" " -f3;rm -f tmp_dME.c)

# for now keep these distinct
GCC_MAJOR_VERSION ?= $(shell $(CC) -v 2>&1 | fgrep 'gcc version' | cut -f3 -d' ' | cut -f1 -d'.')
GCC_MINOR_VERSION ?= $(shell $(CC) -v 2>&1 | fgrep 'gcc version' | cut -f3 -d' ' | cut -f2 -d'.')
