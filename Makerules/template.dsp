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
# Microsoft Developer Studio Project File - Name="ProjectTemplate" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) External Target" 0x0106

CFG=ProjectTemplate - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "ProjectTemplate.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "ProjectTemplate.mak" CFG="ProjectTemplate - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "ProjectTemplate - Win32 Release" (based on "Win32 (x86) External Target")
!MESSAGE "ProjectTemplate - Win32 Debug" (based on "Win32 (x86) External Target")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
!MESSAGE disabled PROP Scc_ProjName "ProjectTemplate"
!MESSAGE disabled PROP Scc_LocalPath "."

!IF  "$(CFG)" == "ProjectTemplate - Win32 Release"

# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Cmd_Line "NMAKE /f ProjectTemplate.mak"
# PROP BASE Rebuild_Opt "/a"
# PROP BASE Target_File "ProjectTemplate.exe"
# PROP BASE Bsc_Name "ProjectTemplate.bsc"
# PROP BASE Target_Dir ""
# PROP Use_Debug_Libraries 0
# PROP Output_Dir ""
# PROP Intermediate_Dir ""
# PROP Cmd_Line "ProjectMake "BUILD_CONFIG=release""
# PROP Rebuild_Opt "clobber stage"
# PROP Target_File "ProjectExecute"
# PROP Bsc_Name "ProjectTemplate.bsc"
# PROP Target_Dir ""

!ELSEIF  "$(CFG)" == "ProjectTemplate - Win32 Debug"

# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Cmd_Line "NMAKE /f ProjectTemplate.mak"
# PROP BASE Rebuild_Opt "/a"
# PROP BASE Target_File "ProjectTemplate.exe"
# PROP BASE Bsc_Name "ProjectTemplate.bsc"
# PROP BASE Target_Dir ""
# PROP Use_Debug_Libraries 1
# PROP Output_Dir ""
# PROP Intermediate_Dir ""
# PROP Cmd_Line "ProjectMake "BUILD_CONFIG=debug""
# PROP Rebuild_Opt "clobber stage"
# PROP Target_File "ProjectExecute"
# PROP Bsc_Name "ProjectTemplate.bsc"
# PROP Target_Dir ""

!ENDIF 

# Begin Target

# Name "ProjectTemplate - Win32 Release"
# Name "ProjectTemplate - Win32 Debug"

!IF  "$(CFG)" == "ProjectTemplate - Win32 Release"

!ELSEIF  "$(CFG)" == "ProjectTemplate - Win32 Debug"

!ENDIF 


