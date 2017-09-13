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
# Makefile for opamgt

# Include Make Control Settings
include $(TL_DIR)/$(PROJ_FILE_DIR)/Makesettings.project

#=============================================================================#
# Definitions:
#-----------------------------------------------------------------------------#

# Name of SubProjects
DS_SUBPROJECTS	= 
# name of executable or downloadable image
EXECUTABLE		= #omgt_tils$(EXE_SUFFIX)
# list of sub directories to build
DIRS			= 
# C files (.c)
CFILES			= \
				src/ib_utils_openib.c \
				src/iba2ibo_util_helper.c  \
				src/ib_notice.c \
				src/ib_notice_net.c \
				src/opamgt_dump_mad.c \
				src/stl_convertfuncs.c \
				src/opamgt.c \
				src/opamgt_pa.c \
				src/opamgt_sa.c \
				src/opamgt_sa_query.c \
				src/omgt_oob_connections.c \
				src/omgt_oob_net.c  \
				src/omgt_oob_net_blob.c \
				src/omgt_oob_net_queue.c \
				src/omgt_oob_ssl.c
				# Add more c files here
# C++ files (.cpp)
CCFILES			= \
				# Add more cpp files here
# lex files (.lex)
LFILES			= \
				# Add more lex files here
# archive library files (basename, $ARFILES will add MOD_LIB_DIR/prefix and suffix)
LIBFILES=
# Windows Resource Files (.rc)
RSCFILES		=
# Windows IDL File (.idl)
IDLFILE			=
# Windows Linker Module Definitions (.def) file for dll's
DEFFILE			=
# targets to build during INCLUDES phase (add public includes here)
INCLUDE_TARGETS	= \
				src/ib_utils_openib.h \
				src/ib_notice_net.h \
				src/stl_convertfuncs.h \
				include/public/opamgt.h \
				include/public/opamgt_pa.h \
				include/public/opamgt_sa.h  \
				include/opamgt_dump_mad.h  \
				include/opamgt_priv.h \
				include/opamgt_pa_priv.h \
				include/opamgt_sa_priv.h \
				src/omgt_oob_ssl.h
				# Add more h hpp files here
# Library example files, to be include during staging
SAMPLE_TARGETS = \
				 samples/README \
				 samples/Makefile \
				 samples/saquery.c \
				 samples/paquery.c \
				 samples/simple_sa_query.c \
				 samples/simple_pa_query.c

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
LIB_TARGETS_ARLIB	= $(LIB_PREFIX)opamgt-priv$(ARLIB_SUFFIX)
LIB_TARGETS_EXP		= $(LIB_TARGETS_IMPLIB:$(ARLIB_SUFFIX)=$(EXP_SUFFIX))
LIB_TARGETS_MISC	= 
# targets to build during CMDS phase
SHLIB_VERSION		= $(shell cat ./version)
CMD_TARGETS_SHLIB	= $(LIB_PREFIX)opamgt$(SHLIB_VERSION_SUFFIX)
vals = $(subst ., ,$(notdir $(CMD_TARGETS_SHLIB)))
override SHLIB_SONAME	= $(word 1,$(vals)).$(word 2,$(vals)).$(word 3,$(vals))
CMD_TARGETS_EXE		= $(EXECUTABLE)
CMD_TARGETS_MISC	= 
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

CLOCAL=$(CIBACCESS) $(CPIC)
LOCALDEPLIBS = 
LOCALDEPLIBS=$(IBACCESS_USER_LIBS)
LOCAL_INCLUDE_DIRS=include . src
LOCALLIBS=$(OPENIB_USER_LIBS)
LOCAL_LIB_DIRS=$(OPENIB_USER_LIB_DIRS) $(IBACCESS_USER_LIB_DIRS)

# Include Make Rules definitions and rules
include $(TL_DIR)/$(PROJ_FILE_DIR)/Makerules.project


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
#include $(TL_DIR)/Makerules/Maketargets.stage
STAGE::
ifeq "$(PRODUCT)" "OPENIB_FF"
		$(VS)$(STAGE_INSTALL) $(STAGE_INSTALL_DIR_OPT) $(PROJ_STAGE_LIB_DIR) $(CMD_TARGETS_SHLIB)
		$(VS)$(STAGE_INSTALL) $(STAGE_INSTALL_DIR_OPT) $(PROJ_STAGE_OPAMGT_DIR) $(INCLUDE_TARGETS)
		$(VS)$(STAGE_INSTALL) $(STAGE_INSTALL_DIR_OPT) $(PROJ_STAGE_OPAMGT_DIR) $(SAMPLE_TARGETS)
endif

# Unit test execution
#include $(TL_DIR)/Makerules/Maketargets.runtest

#=============================================================================#

#=============================================================================#
# DO NOT DELETE THIS LINE -- make depend depends on it.
#=============================================================================#

