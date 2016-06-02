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
ifeq "$(BUILD_TARGET_OS)" "VXWORKS"
DIRS			= src
else
DIRS			= src test man
endif
# C files (.c)
CFILES			= \
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
MISC_FILES		= 
# all source files
SOURCES			= $(CFILES) $(CCFILES) $(LFILES) $(RSCFILES) $(IDLFILE)
# Source files to include in DSP File
DSP_SOURCES		= $(INCLUDE_TARGETS) $(SOURCES) $(MISC_FILES) \
				  $(RSCFILES) $(DEFFILE) $(MAKEFILE) Makerules.module 
# all object files
OBJECTS			= $(CFILES:.c=$(OBJ_SUFFIX)) $(CCFILES:.cpp=$(OBJ_SUFFIX)) \
				  $(LFILES:.lex=$(OBJ_SUFFIX))
RSCOBJECTS		= $(RSCFILES:.rc=$(RES_SUFFIX))
# targets to build during LIBS phase
LIB_TARGETS_IMPLIB	=
ifeq "$(BUILD_TARGET_OS)" "VXWORKS"
LIB_TARGETS_ARLIB	= $(LIB_PREFIX)Esm$(ARLIB_SUFFIX) \
					  #$(LIB_PREFIX)config$(ARLIB_SUFFIX)
else
LIB_TARGETS_ARLIB	= 
endif
LIB_TARGETS_EXP		= $(LIB_TARGETS_IMPLIB:$(ARLIB_SUFFIX)=$(EXP_SUFFIX))
LIB_TARGETS_MISC	= 
# targets to build during CMDS phase
CMD_TARGETS_SHLIB	= 
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

ifeq "$(BUILD_TARGET_OS)" "VXWORKS"
#LOCALDEPLIBS = cs fe if3 if3sa mai net pm sa sm vxworks
LOCALDEPLIBS = vxworks mai cs if3 if3sa pm sm sa config fe net 
endif

LOCAL_INCLUDE_DIRS = $(TL_DIR)/Topology $(TL_DIR)/IbPrint

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
#include $(TL_DIR)/Makerules/Maketargets.build

ifeq "$(BUILD_TARGET_OS)" "VXWORKS"
# build libEsm.a from the set of module libraries by taking them apart
# and then putting them together into libEsm.a
CT_LIBS=$(foreach f,$(DEPLIBS_TARGETS),$(MOD_LIB_DIR)/$(f))
$(LIB_PREFIX)Esm$(ARLIB_SUFFIX): $(CT_LIBS)
	@mkdir -p $(dir $@)
	rm -rf $(BUILDDIR)/libEsm
	mkdir -p $(BUILDDIR)/libEsm
	cd $(BUILDDIR)/libEsm/; \
		for i in $(CT_LIBS); \
		do \
			$(AR) xv $$i; \
		done
	$(VS)$(MKARLIB)$@ $(BUILDDIR)/libEsm/*

CLEAN::
	rm -rf $(BUILDDIR)/libEsm
endif

# install for includes, libs and cmds phases
include $(TL_DIR)/Makerules/Maketargets.moduleinstall

# install for stage phase
ifeq "$(BUILD_TARGET_OS)" "VXWORKS"
STAGE::
else
#include $(TL_DIR)/Makerules/Maketargets.modulestage
# create the opafm rpm
# lower case platform name
platform=$(shell echo $(BUILD_TARGET)| tr '[A-Z]' '[a-z]')
package:
	rm -rf $(PROJ_STAGE_PACKAGES_DIR)
	mkdir -p $(PROJ_STAGE_PACKAGES_DIR)
	mkdir -p $(PROJ_STAGE_PACKAGES_DIR)/RPMS/$(platform)
	mkdir -p $(PROJ_STAGE_PACKAGES_DIR)/BUILD
	mkdir -p $(PROJ_STAGE_PACKAGES_DIR)/BUILDROOT
	mkdir -p $(PROJ_STAGE_PACKAGES_DIR)/SOURCES
	mkdir -p $(PROJ_STAGE_PACKAGES_DIR)/SPECS
	mkdir -p $(PROJ_STAGE_PACKAGES_DIR)/SRPMS
	sed -e "s/^Version:.*/Version:\ $(RPM_REL)/g" \
		-e "s/^Release:.*/Release:\ $(RPM_VER)/g" \
		-e "s/^Source:.*/Source:\ $(RPM_NAME)-$(RPM_REL).tar.gz/g" \
		< src/linux/rpm/vieo_fm.spec.base > $(PROJ_STAGE_PACKAGES_DIR)/SPECS/$(RPM_NAME).spec
	# Append list of files to be installed to spec file
	# strip the top part of the path away and replace it with the install dir.
	cd $(PROJ_STAGE_IMAGE_DIR); find -type f -o -type l | sort | sed 's/./\/opt\/opafm/' >> $(PROJ_STAGE_PACKAGES_DIR)/SPECS/$(RPM_NAME).spec
	cd $(STAGE_DIR) ; \
		tar cvfz $(PROJ_STAGE_PACKAGES_DIR)/SOURCES/$(RPM_NAME)-$(RPM_REL).tar.gz $(RPM_NAME)-$(RPM_REL)
	rpmbuild \
		--buildroot $(PROJ_STAGE_PACKAGES_DIR)/BUILDROOT \
		--define '_unpackaged_files_terminate_build 1' \
		--define 'debug_package	%{nil}' \
		--define 'dist %{nil}' \
		--define '__spec_build_pre %{___build_pre}' \
		--define "_topdir $(PROJ_STAGE_PACKAGES_DIR)" \
		 	-bb $(PROJ_STAGE_PACKAGES_DIR)/SPECS/$(RPM_NAME).spec
	cp $(PROJ_STAGE_PACKAGES_DIR)/RPMS/$(platform)/$(RPM_NAME)-$(RPM_REL)-$(RPM_VER).$(platform).rpm $(PROJ_STAGE_DIR)/$(RPM_NAME)-$(RPM_REL)_$(RPM_VER).$(platform).rpm
endif

# Unit test execution
#include $(TL_DIR)/Makerules/Maketargets.runtest

clobber:: clobber_module

#=============================================================================#

#=============================================================================#
# DO NOT DELETE THIS LINE -- make depend depends on it.
#=============================================================================#
