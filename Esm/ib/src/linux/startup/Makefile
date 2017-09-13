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
# Makefile for SM Module

# Include Make Control Settings
include $(TL_DIR)/$(PROJ_FILE_DIR)/Makesettings.project

#=============================================================================#
# Definitions:
#-----------------------------------------------------------------------------#

# Name of SubProjects
DS_SUBPROJECTS	= 
# name of executable or downloadable image
EXECUTABLE		= # Sm$(EXE_SUFFIX)
# list of sub directories to build
DIRS			= fm_config
# C files (.c)
CFILES			= opafmd.c
				# Add more c files here
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
				# Add more h hpp files here
# Non-compiled files
MISC_FILES		= opafm.service opafe@.service opasm@.service opafm_src.xml
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
#LIB_TARGETS_ARLIB	= $(LIB_PREFIX)name$(ARLIB_SUFFIX)
LIB_TARGETS_ARLIB	= 
LIB_TARGETS_EXP		= $(LIB_TARGETS_IMPLIB:$(ARLIB_SUFFIX)=$(EXP_SUFFIX))
LIB_TARGETS_MISC	= 
# targets to build during CMDS phase
CMD_TARGETS_SHLIB	= 
CMD_TARGETS_EXE		= $(EXECUTABLE)
CMD_TARGETS_MISC	= fm_cmdall opafm fm_capture \
				  opafm.xml $(BUILDDIR)/opaxmlextract $(BUILDDIR)/opaxmlfilter \
				  $(BUILDDIR)/opafmd opafmctrl 
# files to remove during clean phase
CLEAN_TARGETS_MISC	=  
CLEAN_TARGETS		= $(OBJECTS) $(RSCOBJECTS) $(IDL_TARGETS) $(CLEAN_TARGETS_MISC)
# other files to remove during clobber phase
CLOBBER_TARGETS_MISC= sm_capture
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

CLOCAL	=  $(CPIE)
LOCALDEPLIBS = cs ibaccess public config vslogu Xml expat Md5 opamgt-priv
LOCALLIBS = pthread $(OPENIB_USER_LIBS) rt

# Include Make Rules definitions and rules
include $(PROJ_SM_DIR)/Makerules.module

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

$(BUILDDIR)/opafmd: $(BUILDDIR)/opafmd.o
	$(VS)$(CC) -pthread $(LDFLAGS)$@ $(BUILDDIR)/opafmd.o $(LDLIBS)

$(BUILDDIR)/opaxmlextract: $(PROJ_BIN_DIR)/opaxmlextract
	cp $(PROJ_BIN_DIR)/opaxmlextract $(BUILDDIR)

$(BUILDDIR)/opaxmlfilter: $(PROJ_BIN_DIR)/opaxmlfilter
	cp $(PROJ_BIN_DIR)/opaxmlfilter $(BUILDDIR)

fm_capture: sm_capture
	cp sm_capture fm_capture

$(BUILDDIR)/config_convert: $(PROJ_BIN_DIR)/config_convert
	cp $(PROJ_BIN_DIR)/config_convert $(BUILDDIR)

opafm.xml : opafm_src.xml $(BUILDDIR)/config_convert
	$(BUILDDIR)/config_convert dummy.config opafm_src.xml > opafm.xml

opafmctrl: opafmctrl.sh
	cp opafmctrl.sh opafmctrl

# install for stage phase
#include $(TL_DIR)/Makerules/Maketargets.stage
STAGE::
	$(VS)$(STAGE_INSTALL) $(STAGE_INSTALL_DIR_OPT) $(PROJ_STAGE_DIR)/etc \
		opafm_src.xml
	$(VS)$(STAGE_INSTALL) $(STAGE_INSTALL_DIR_OPT) $(PROJ_STAGE_IMAGE_DIR)/bin \
		fm_capture fm_cmdall opafmctrl
	$(VS)$(STAGE_INSTALL) $(STAGE_INSTALL_DIR_OPT) $(PROJ_STAGE_IMAGE_DIR)/etc \
		opafm \
		opafm.xml $(BUILDDIR)/opaxmlextract $(BUILDDIR)/opaxmlfilter \
		opafm_src.xml

# Unit test execution
#include $(TL_DIR)/Makerules/Maketargets.runtest

clobber:: clobber_module

#=============================================================================#

#=============================================================================#
# DO NOT DELETE THIS LINE -- make depend depends on it.
#=============================================================================#
