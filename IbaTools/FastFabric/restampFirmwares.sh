#!/bin/bash
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

#[ICS VERSION STRING: unknown]
# script to restamp embedded firmware with different version numbers to aid
# run_tests scripts for firmware push

#NEWVERSION=3_1_0_0_29S1
NEWVERSION=cking2

#SOURCEDIR=~ftp/Integration/GA3_1_0_0/ALL_EMB/3_1_0_0_29/Infinicon
SOURCEDIR=cking/

# product codes
EIOU_PC=1
FCIOU_PC=2
CMU_PC=3
IBX_PC=4
I2K_PC=5
I3K_PC=6
I5K_PC=7
I9K_PC=8

# bsp codes
SWAUX_BC=3
PCIX_BC=7
MC1125_BC=9
T3_BC=10

STRIP=/usr/local/bin/mips64-vxworks-strip
DEFLATE=/usr/local/ics/bin/icsdeflate


mkdir $NEWVERSION
cd $NEWVERSION

for i in CMU/CMU.sw_aux CMU/CMU.mc1125 EIOU/EIOU.pcix FCIOU/FCIOU.pcix IBX/IBX.sw_aux IBX/IBX.mc1125 I2K/I2K.mc1125 I3K/I3K.mc1125 I5K/I5K.t3 I9K/I9K.t3
do
	cp $SOURCEDIR/$i .
done

for i in CMU.sw_aux CMU.mc1125 EIOU.pcix FCIOU.pcix IBX.sw_aux IBX.mc1125 I2K.mc1125 I3K.mc1125 I5K.t3 I9K.t3
do
	patch_version -n `format_releasetag $NEWVERSION` $NEWVERSION $i
done

mkpkg CMU.sw_aux InfiniFabric sw_aux $CMU_PC $SWAUX_BC $STRIP $DEFLATE
mkpkg CMU.mc1125 InfiniFabric mc1125 $CMU_PC $MC1125_BC $STRIP $DEFLATE
mkpkg EIOU.pcix VEx pcix $EIOU_PC $PCIX_BC $STRIP $DEFLATE
mkpkg FCIOU.pcix VFx pcix $FCIOU_PC $PCIX_BC $STRIP $DEFLATE
mkpkg IBX.sw_aux IBx sw_aux $IBX_PC $SWAUX_BC $STRIP $DEFLATE
mkpkg IBX.mc1125 IBx mc1125 $IBX_PC $MC1125_BC $STRIP $DEFLATE
mkpkg I2K.mc1125 InfinIO2000 mc1125 $I2K_PC $MC1125_BC $STRIP $DEFLATE
mkpkg I3K.mc1125 InfinIO3000 mc1125 $I3K_PC $MC1125_BC $STRIP $DEFLATE
mkpkg I5K.t3 InfinIO5000 t3 $I5K_PC $T3_BC $STRIP $DEFLATE
mkpkg I9K.t3 InfinIO9000 t3 $I9K_PC $T3_BC $STRIP $DEFLATE

rm -f CMU.* EIOU.* FCIOU.* IBX.* I[2359]K.*
