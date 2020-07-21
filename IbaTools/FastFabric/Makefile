# BEGIN_ICS_COPYRIGHT8 ****************************************
#
# Copyright (c) 2015-2020, Intel Corporation
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

# Makefile for cluster_tools

# Include Make Control Settings
include $(TL_DIR)/$(PROJ_FILE_DIR)/Makesettings.project

#=============================================================================#
# Definitions:
#-----------------------------------------------------------------------------#

# Name of SubProjects
DS_SUBPROJECTS	= 
# name of executable or downloadable image
EXECUTABLE		= #$(BUILDDIR)/FastFabric$(EXE_SUFFIX)
# list of sub directories to build
DIRS			= 
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
LIBFILES=
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
MISC_FILES		= switches allhosts-sample chassis-sample esm_chassis-sample \
				hosts-sample switches-sample ports-sample osid_wrapper \
				mac_to_dhcp opa_ca_openssl.cnf opa_comp_openssl.cnf \
				opa_ca_openssl.cnf-sample opa_comp_openssl.cnf-sample
# expect files
TCL_SOURCES		= chassis_reboot.exp reboot.exp sacache.exp load.exp \
				chassis_upgrade.exp chassis_fmconfig.exp chassis_fmcontrol.exp \
				chassis_fmgetconfig.exp chassis_fmgetsecurityfiles.exp \
				chassis_fmsecurityfiles.exp upgrade.exp configipoib.exp \
				mpiperf.exp mpiperfdeviation.exp ipoibping.exp ff_function.exp \
				switch_upgrade.exp switch_dump.exp switch_reboot.exp \
				switch_info.exp switch_hwvpd.exp switch_ping.exp \
				switch_fwverify.exp switch_capture.exp chassis_configure.exp \
				switch_configure.exp switch_getconfig.exp chassis_getconfig.exp
# all source files
SOURCES			= $(CFILES) $(CCFILES) $(LFILES) $(RSCFILES) $(IDLFILE)
# Source files to include in DSP File
DSP_SOURCES		= $(INCLUDE_TARGETS) $(SOURCES) $(MISC_FILES) \
				  $(RSCFILES) $(DEFFILE) $(MAKEFILE)  $(TCL_SOURCES)
# all object files
OBJECTS			= $(CFILES:.c=$(OBJ_SUFFIX)) $(CCFILES:.cpp=$(OBJ_SUFFIX)) \
				  $(LFILES:.lex=$(OBJ_SUFFIX))
RSCOBJECTS		= $(RSCFILES:.rc=$(RES_SUFFIX))
# targets to build during LIBS phase
LIB_TARGETS_IMPLIB	=
LIB_TARGETS_ARLIB	= # $(LIB_PREFIX)ResourceTest$(ARLIB_SUFFIX)
LIB_TARGETS_EXP		= $(LIB_TARGETS_IMPLIB:$(ARLIB_SUFFIX)=$(EXP_SUFFIX))
LIB_TARGETS_MISC	= 
# targets to build during CMDS phase
CMD_TARGETS_SHLIB	= 
CMD_TARGETS_EXE		= $(EXECUTABLE)
CMD_TARGETS_MISC	= opasetupssh opascpall opauploadall opadownloadall opapingall opacmdall opashowallports opacaptureall opahostadmin opachassisadmin opaswitchadmin tcl_proc ff_funcs opafastfabric.conf.def opafastfabric.conf-sample opafabricanalysis opahostsmanalysis opaesmanalysis opachassisanalysis opaallanalysis opafastfabric opaswenableall opaswdisableall opaexpandfile show_counts opacablehealthcron opa-cablehealth
# files to remove during clean phase
CLEAN_TARGETS_MISC  = opatest
CLEAN_TARGETS       = $(OBJECTS) $(RSCOBJECTS) $(IDL_TARGETS) $(CLEAN_TARGETS_MISC)
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

CLOCAL = $(CPIE)

# Include Make Rules definitions and rules
include $(TL_DIR)/IbaTools/Makerules.module

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
#$(BUILDDIR)/opafastfabric.conf: opafastfabric.conf.$(BUILD_TARGET_OS).$(IB_STACK) $(BUILDDIR)
opafastfabric.conf.def: opafastfabric.conf.$(BUILD_TARGET_OS).$(IB_STACK)
	if [ "$(BUILD_TARGET_OS_VENDOR_VERSION)" = "ES123" ]; then \
		sed -i '/FF_IPOIB_CONNECTED=/s/no/yes/' $<; \
	fi;
	$(CP) $< $@
	chmod a+x $@

opafastfabric.conf-sample: opafastfabric.conf.$(BUILD_TARGET_OS).$(IB_STACK)
	$(CP) $< $@
	chmod a+x $@

.SUFFIXES: .pl -sample
%:%.pl
	$(RM) $(RMFORCE) $@
	$(CP) $< $@

%-sample:%
	$(CP) $< $@

opaswenableall: opaswdisableall.sh
	$(CP) $< $@

opafastfabric: opafastfabric.pl
	$(SED) -e 's/THIS_IS_THE_IB_STACK_TYPE/$(IB_STACK)/' < $< > $@
	$(CHMOD) a+x $@

opahostadmin: opatest
	$(CP) $< $@

opachassisadmin: opatest
	$(CP) $< $@

opaswitchadmin: opatest
	$(CP) $< $@

opacablehealthcron: opacablehealthcron.sh
	$(CP) $< $@

opa-cablehealth: opacablehealth.cron
	$(CP) $< $@

switches:	ibnodes
	$(CP) $< $@

osid_wrapper:
	echo $(BUILD_TARGET_OS_ID) > osid_wrapper

# process Sub-directories
include $(TL_DIR)/Makerules/Maketargets.toplevel

# build cmds and libs
include $(TL_DIR)/Makerules/Maketargets.build

# install for includes, libs and cmds phases
include $(TL_DIR)/Makerules/Maketargets.install

# install for stage phase
#include $(TL_DIR)/Makerules/Maketargets.stage
STAGE::
	$(VS)$(STAGE_INSTALL) $(STAGE_INSTALL_DIR_OPT) $(PROJ_STAGE_FASTFABRIC_DIR) opasetupssh opascpall opauploadall opadownloadall opapingall opacmdall opashowallports opacaptureall opahostadmin opachassisadmin opaswitchadmin opafabricanalysis opahostsmanalysis opaesmanalysis opachassisanalysis opaallanalysis opafastfabric opaswenableall opaswdisableall opaexpandfile ff_funcs opafastfabric.conf.def show_counts opacablehealthcron
	$(VS)$(STAGE_INSTALL) $(STAGE_INSTALL_DIR_OPT) $(PROJ_STAGE_IBTOOLS_DIR) tcl_proc $(TCL_SOURCES) opafastfabric.conf.def ff_funcs ports hosts allhosts switches chassis esm_chassis osid_wrapper
	$(VS)$(STAGE_INSTALL) $(STAGE_INSTALL_DIR_OPT) $(PROJ_STAGE_FASTFABRIC_DIR)/samples allhosts-sample chassis-sample hosts-sample switches-sample ports-sample mac_to_dhcp opafastfabric.conf-sample opa_ca_openssl.cnf-sample opa_comp_openssl.cnf-sample esm_chassis-sample
	$(VS)$(STAGE_INSTALL) $(STAGE_INSTALL_DIR_OPT) $(PROJ_STAGE_ETC_DIR)/cron.d opa-cablehealth

# Unit test execution
#include $(TL_DIR)/Makerules/Maketargets.runtest

#=============================================================================#

#=============================================================================#
# DO NOT DELETE THIS LINE -- make depend depends on it.
#=============================================================================#
