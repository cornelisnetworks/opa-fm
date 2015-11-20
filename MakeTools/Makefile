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
# Makefile for MakeTools

# Include Make Control Settings
include $(TL_DIR)/$(PROJ_FILE_DIR)/Makesettings.project

DS_SUBPROJECTS =
DSP_SOURCES= Makefile \
			buildmsgs.pl \
			build_srpms.sh \
			expand_source.sh \
			expand_ofed.sh \
			get_ofed_files.sh \
			getsmpopt.sh \
			get_comp.sh \
			get_input_comp_files.sh \
			lnshlib.sh \
			lnshlib.darwin.sh \
			ln-install.sh \
			makeiso.sh \
			mkarlib.sh \
			builddsw.sh \
			finddspfiles.sh \
			makewin32depend.sh \
			prep.sh
DIRS		= \
			patch_version \
			installtargets \
			stripdepends \
			rm_version \
			$(TEST)
CMD_TARGETS_MISC = \
			builddsw \
			build_srpms \
			expand_source \
			expand_ofed \
			get_ofed_files \
			get_comp \
			get_input_comp_files \
			finddspfiles \
			ln-install \
			makewin32depend
ifeq "$(PRODUCT)" "devtools"
CMD_TARGETS_MISC += 	prep \
			patch_engineer_version \
			convert_releasetag
endif

INCLUDE_TARGETS		= 

include $(TL_DIR)/$(PROJ_FILE_DIR)/Makerules.project

include $(TL_DIR)/Makerules/Maketargets.toplevel

include $(TL_DIR)/Makerules/Maketargets.build
convert_releasetag: convert_releasetag.pl
	cp $< $@
	chmod 755 $@

include $(TL_DIR)/Makerules/Maketargets.stage

include $(TL_DIR)/Makerules/Maketargets.install

