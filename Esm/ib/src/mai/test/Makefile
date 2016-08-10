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

# $Id$

include $(shell GetBuildRoot)/ib/rules/master.cf

MF_FLAGS	= -D_REENTRANT -D_THREAD_SAFE -DPAW -DPAW3


FMR_SRC		= server.c
FMR_LIB		= -Xlinker --start-group $(LIB_MAI) $(LIB_CS) $(LIB_MAI) $(LIB_OPENIB) -lpthread -Xlinker --end-group

FER_SRC		= client.c
FER_LIB		= -Xlinker --start-group $(LIB_MAI) $(LIB_CS) $(LIB_MAI) $(LIB_OPENIB) -lpthread -Xlinker --end-group

TARGETS		= FMR FER

INIT:
			@mkdir -p bin
			@$(DEP_LIB_MAI)
			@$(DEP_LIB_IF3)
			@$(DEP_LIB_SIM)
			@$(DEP_LIB_CS)

FMR:		$(FMR_OBJ)
			$(CC) $(LDFLAGS) $(CFLAGS) -o bin/server $(FMR_OBJ) $(FMR_LIB)

FER:		$(FER_OBJ)
			$(CC) $(LDFLAGS) $(CFLAGS) -o bin/client $(FER_OBJ) $(FER_LIB)

CLEAN:
			rm -fr bin
