# Microsoft Developer Studio Project File - Name="Cmlib" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Generic Project" 0x010a

CFG=Cmlib - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "Cmlib.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "Cmlib.mak" CFG="Cmlib - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "Cmlib - Win32 Release" (based on "Win32 (x86) Generic Project")
!MESSAGE "Cmlib - Win32 Debug" (based on "Win32 (x86) Generic Project")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
MTL=midl.exe

!IF  "$(CFG)" == "Cmlib - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""

!ELSEIF  "$(CFG)" == "Cmlib - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir ""
# PROP Intermediate_Dir ""
# PROP Target_Dir ""

!ENDIF 

# Begin Target

# Name "Cmlib - Win32 Release"
# Name "Cmlib - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "*.c"
# Begin Source File

SOURCE=.\cm_active.c
# End Source File
# Begin Source File

SOURCE=.\cm_api.c
# End Source File
# Begin Source File

SOURCE=.\cm_gsa.c
# End Source File
# Begin Source File

SOURCE=.\cm_msg.c
# End Source File
# Begin Source File

SOURCE=.\cm_passive.c
# End Source File
# Begin Source File

SOURCE=.\cm_test.c
# End Source File
# Begin Source File

SOURCE=.\sidr_api.c
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "*.h"
# Begin Source File

SOURCE=.\cm_common.h
# End Source File
# Begin Source File

SOURCE=.\cm_msg.h
# End Source File
# Begin Source File

SOURCE=.\cm_private.h
# End Source File
# Begin Source File

SOURCE=..\..\Inc\ib_cm.h
# End Source File
# End Group
# Begin Source File

SOURCE=.\CM_notes.txt
# End Source File
# Begin Source File

SOURCE=.\CM_readme.txt
# End Source File
# End Target
# End Project
