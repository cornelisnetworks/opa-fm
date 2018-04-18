# BEGIN_ICS_COPYRIGHT8 ****************************************
# 
# Copyright (c) 2015-2017, Intel Corporation
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
# Makefile for public

# Include Make Control Settings
include $(TL_DIR)/$(PROJ_FILE_DIR)/Makesettings.project

#=============================================================================#
# Definitions:
#-----------------------------------------------------------------------------#

# Name of SubProjects
DS_SUBPROJECTS	= 
# name of executable or downloadable image
EXECUTABLE		= # public$(EXE_SUFFIX)
# list of sub directories to build
DIRS			= 
# C files (.c)
CFILES			= \
				iarray.c \
				ibitvector.c \
				icmdthread.c \
				idebugdump.c \
				idebug_linux.c \
				ievent.c \
				ieventthread.c \
				ilist.c \
				imath.c \
				imemory.c \
				imemory_osd.c \
				iobjmgr.c \
				iquickmap.c \
				isemaphore.c \
				ispinlock.c \
				ithread.c \
				itimer.c \
				statustext.c \
				# Add more c files here
				# ireaper.c \

CFILES			+= \
# C++ files (.cpp)
CCFILES			= \
				# Add more cpp files here
# lex files (.lex)
LFILES			= \
				# Add more lex files here
# archive library files (basename, $ARFILES will add MOD_LIB_DIR/prefix and suffix)
LIBFILES = 
# Windows Resource Files (.rc)
RSCFILES		=
# Windows IDL File (.idl)
IDLFILE			=
# Windows Linker Module Definitions (.def) file for dll's
DEFFILE			=
# targets to build during INCLUDES phase (add public includes here)
INCLUDE_TARGETS	= \
				$(COMMON_SRCDIR)/datatypes.h \
				$(COMMON_SRCDIR)/iarray.h \
				$(COMMON_SRCDIR)/ibitvector.h \
				$(COMMON_SRCDIR)/ibyteswap.h \
				$(COMMON_SRCDIR)/icmdthread.h \
				$(COMMON_SRCDIR)/idebug.h \
				$(COMMON_SRCDIR)/iethernet.h \
				$(COMMON_SRCDIR)/ievent.h \
				$(COMMON_SRCDIR)/ieventthread.h \
				$(COMMON_SRCDIR)/iheapmanager.h \
				$(COMMON_SRCDIR)/ilist.h \
				$(COMMON_SRCDIR)/imath.h \
				$(COMMON_SRCDIR)/imemory.h \
				$(COMMON_SRCDIR)/imemtrack.h \
				$(COMMON_SRCDIR)/imutex.h \
				$(COMMON_SRCDIR)/iobjmgr.h \
				$(COMMON_SRCDIR)/iquickmap.h \
				$(COMMON_SRCDIR)/isemaphore.h \
				$(COMMON_SRCDIR)/ispinlock.h \
				$(COMMON_SRCDIR)/itcpip.h \
				$(COMMON_SRCDIR)/ithread.h \
				$(COMMON_SRCDIR)/itimer.h \
				$(COMMON_SRCDIR)/statustext.h \
				datatypes_osd.h \
				idebug_osd.h \
				ievent_osd.h \
				imemory_osd.h \
				imutex_osd.h \
				ipackoff.h \
				ipackon.h \
				isemaphore_osd.h \
				ispinlock_osd.h \
				ithread_osd.h \
				itimer_osd.h \
				ibyteswap_osd.h
				# Add more h hpp files here
				# $(COMMON_SRCDIR)/ipassivelock.h \
				# $(COMMON_SRCDIR)/ireaper.h \

# Non-compiled files
MISC_FILES		= 
# all source files
SOURCES			= $(CFILES) $(CCFILES) $(LFILES) $(RSCFILES) $(IDLFILE)
# Source files to include in DSP File
DSP_SOURCES		= $(INCLUDE_TARGETS) $(SOURCES) $(MISC_FILES) \
				  $(RSCFILES) $(DEFFILE) $(MAKEFILE) 
# all object files
OBJECTS			= $(CFILES:.c=$(OBJ_SUFFIX)) $(CCFILES:.cpp=$(OBJ_SUFFIX)) \
				  $(LFILES:.lex=$(OBJ_SUFFIX))
RSCOBJECTS		= $(RSCFILES:.rc=$(RES_SUFFIX))
# targets to build during LIBS phase
LIB_TARGETS_IMPLIB	=
ifeq "$(IB_STACK)" "OPENIB"
LIB_TARGETS_ARLIB	= $(LIB_PREFIX)public$(ARLIB_SUFFIX)
else
LIB_TARGETS_ARLIB	= 
endif
LIB_TARGETS_EXP		= $(LIB_TARGETS_IMPLIB:$(ARLIB_SUFFIX)=$(EXP_SUFFIX))
LIB_TARGETS_MISC	= 
# targets to build during CMDS phase
SHLIB_VERSION		= 0.0
ifeq "$(IB_STACK)" "OPENIB"
CMD_TARGETS_SHLIB	= 
else
CMD_TARGETS_SHLIB	= $(LIB_PREFIX)public$(SHLIB_VERSION_SUFFIX)
endif
CMD_TARGETS_EXE		= $(EXECUTABLE)
CMD_TARGETS_MISC	=
CMD_TARGETS_DRIVER	= 
# files to remove during clean phase
CLEAN_TARGETS_MISC	=  
CLEAN_TARGETS		= $(OBJECTS) $(RSCOBJECTS) $(IDL_TARGETS) $(CLEAN_TARGETS_MISC)
# other files to remove during clobber phase
CLOBBER_TARGETS_MISC=
# sub-directory to install to within bin
BIN_SUBDIR		= 
# sub-directory to install to within include
INCLUDE_SUBDIR		=

# Additional Settings
#CLOCALDEBUG	= User defined C debugging compilation flags [Empty]
#CCLOCALDEBUG	= User defined C++ debugging compilation flags [Empty]
#CLOCAL	= User defined C flags for compiling [Empty]
#CCLOCAL	= User defined C++ flags for compiling [Empty]
#BSCLOCAL	= User flags for Browse File Builder [Empty]
#DEPENDLOCAL	= user defined makedepend flags [Empty]
#LINTLOCAL	= User defined lint flags [Empty]
#LOCAL_INCLUDE_DIRS	= User include directories to search for C/C++ headers [Empty]
#LDLOCAL	= User defined C flags for linking [Empty]
#IMPLIBLOCAL	= User flags for Object Lirary Manager [Empty]
#MIDLLOCAL	= User flags for IDL compiler [Empty]
#RSCLOCAL	= User flags for resource compiler [Empty]
#LOCALDEPLIBS	= User libraries to include in dependencies [Empty]
#LOCALLIBS		= User libraries to use when linking [Empty]
#				(in addition to LOCALDEPLIBS)
#LOCAL_LIB_DIRS	= User library directories for libpaths [Empty]

LOCALDEPLIBS = 
# need -nostartfiles since we have our own _init
LDLOCAL=-nostartfiles

# Include Make Rules definitions and rules
include $(TL_DIR)/IbAccess/UserLinux/Makerules.module

COMMON_SRCDIR = $(MOD_DIR)/../Common/Public
MOD_INCLUDE_SUBDIR = iba/public

#=============================================================================#
# Overrides:
#-----------------------------------------------------------------------------#
#CCOPT			=	# C++ optimization flags, default lets build config decide
#COPT			=	# C optimization flags, default lets build config decide
#SUBSYSTEM = Subsystem to build for (none, console or windows) [none]
#					 (Windows Only)
#USEMFC	= How Windows MFC should be used (none, static, shared, no_mfc) [none]
#				(Windows Only)
#=============================================================================#

#=============================================================================#
# Rules:
#-----------------------------------------------------------------------------#
# process Sub-directories
include $(TL_DIR)/Makerules/Maketargets.toplevel

# build cmds and libs
include $(TL_DIR)/Makerules/Maketargets.build

# install for includes, libs and cmds phases
include $(TL_DIR)/Makerules/Maketargets.install

# install for stage phase
ifeq "$(IB_STACK)" "OPENIB"
STAGE::
ifeq "$(PRODUCT)" "OPENIB_FF"
	$(VS)$(STAGE_INSTALL) $(STAGE_INSTALL_DIR_OPT) $(PROJ_STAGE_OPAMGT_DIR) $(INCLUDE_TARGETS)
endif
else
include $(TL_DIR)/Makerules/Maketargets.stage
endif

# Unit test execution
#include $(TL_DIR)/Makerules/Maketargets.runtest

#=============================================================================#

#=============================================================================#
# DO NOT DELETE THIS LINE -- make depend depends on it.
#=============================================================================#
