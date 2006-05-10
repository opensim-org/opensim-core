# Microsoft Developer Studio Project File - Name="suAnalyses" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Dynamic-Link Library" 0x0102

CFG=suAnalyses - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "suAnalyses.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "suAnalyses.mak" CFG="suAnalyses - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "suAnalyses - Win32 Release" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE "suAnalyses - Win32 Debug" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
F90=df.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "suAnalyses - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE F90 /compile_only /dll /nologo /warn:nofileopt
# ADD F90 /compile_only /dll /nologo /threads /warn:nofileopt
# ADD BASE CPP /nologo /MT /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /D "SUANALYSES_EXPORTS" /YX /FD /c
# ADD CPP /nologo /MD /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /D "SUANALYSES_EXPORTS" /FD /c
# SUBTRACT CPP /YX
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /dll /machine:I386
# ADD LINK32 rdTools.lib rdSimulation.lib rdSQP.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /dll /machine:I386
# Begin Special Build Tool
SOURCE="$(InputPath)"
PostBuild_Desc=Updating libraries...
PostBuild_Cmds=$(RDI_INSTALL)\Bin\cpl
# End Special Build Tool

!ELSEIF  "$(CFG)" == "suAnalyses - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE F90 /check:bounds /compile_only /debug:full /dll /nologo /traceback /warn:argument_checking /warn:nofileopt
# ADD F90 /check:bounds /compile_only /dbglibs /debug:none /dll /nologo /threads /traceback /warn:argument_checking /warn:nofileopt
# ADD BASE CPP /nologo /MTd /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /D "SUANALYSES_EXPORTS" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /D "SUANALYSES_EXPORTS" /FD /GZ /c
# SUBTRACT CPP /YX
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /dll /debug /machine:I386 /pdbtype:sept
# ADD LINK32 rdTools_D.lib rdSimulation_D.lib rdSQP_D.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /dll /debug /machine:I386 /out:"Debug/suAnalyses_D.dll" /pdbtype:sept
# Begin Special Build Tool
SOURCE="$(InputPath)"
PostBuild_Desc=Updating libraries...
PostBuild_Cmds=$(RDI_INSTALL)\Bin\cpl D
# End Special Build Tool

!ENDIF 

# Begin Target

# Name "suAnalyses - Win32 Release"
# Name "suAnalyses - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat;f90;for;f;fpp"
# Begin Source File

SOURCE=..\InvestigationForward.cpp
# End Source File
# Begin Source File

SOURCE=..\InvestigationPerturbation.cpp
# End Source File
# Begin Source File

SOURCE=..\RegisterTypes_suAnalyses.cpp
# End Source File
# Begin Source File

SOURCE=..\suActuation.cpp
# End Source File
# Begin Source File

SOURCE=..\suActuatorGeneralizedForces.cpp
# End Source File
# Begin Source File

SOURCE=..\suActuatorPerturbation.cpp
# End Source File
# Begin Source File

SOURCE=..\suActuatorPerturbationIndependent.cpp
# End Source File
# Begin Source File

SOURCE=..\suAnalyses.cpp
# End Source File
# Begin Source File

SOURCE=..\suAnalysesDLL.cpp
# End Source File
# Begin Source File

SOURCE=..\suBodyIndAcc.cpp
# End Source File
# Begin Source File

SOURCE=..\suBodyIndAccCOM.cpp
# End Source File
# Begin Source File

SOURCE=..\suBodyIndPowers.cpp
# End Source File
# Begin Source File

SOURCE=..\suBodyKinematics.cpp
# End Source File
# Begin Source File

SOURCE=..\suBodyPointIndAcc.cpp
# End Source File
# Begin Source File

SOURCE=..\suContact.cpp
# End Source File
# Begin Source File

SOURCE=..\suDecomp.cpp
# End Source File
# Begin Source File

SOURCE=..\suDecompHard.cpp
# End Source File
# Begin Source File

SOURCE=..\suDecompHardUTWalk.cpp
# End Source File
# Begin Source File

SOURCE=..\suDecompHardUTWalkNoComp.cpp
# End Source File
# Begin Source File

SOURCE=..\suDecompHardUTWalkNoCompPrescribed.cpp
# End Source File
# Begin Source File

SOURCE=..\suDecompInteg.cpp
# End Source File
# Begin Source File

SOURCE=..\suDecompNoComp.cpp
# End Source File
# Begin Source File

SOURCE=..\suDecompTarget.cpp
# End Source File
# Begin Source File

SOURCE=..\suDecompTargetNoComp.cpp
# End Source File
# Begin Source File

SOURCE=..\suDecompTargetNoCompPrescribed.cpp
# End Source File
# Begin Source File

SOURCE=..\suDecompTaylor.cpp
# End Source File
# Begin Source File

SOURCE=..\suForceApplier.cpp
# End Source File
# Begin Source File

SOURCE=..\suGeneralizedForceApplier.cpp
# End Source File
# Begin Source File

SOURCE=..\suGeneralizedForcePerturbation.cpp
# End Source File
# Begin Source File

SOURCE=..\suGeneralizedForces.cpp
# End Source File
# Begin Source File

SOURCE=..\suIndAcc.cpp
# End Source File
# Begin Source File

SOURCE=..\suIndContactPowers.cpp
# End Source File
# Begin Source File

SOURCE=..\suKinematics.cpp
# End Source File
# Begin Source File

SOURCE=..\suLinearSpring.cpp
# End Source File
# Begin Source File

SOURCE=..\suPointKinematics.cpp
# End Source File
# Begin Source File

SOURCE=..\suTorqueApplier.cpp
# End Source File
# Begin Source File

SOURCE=..\suTorsionalSpring.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl;fi;fd"
# Begin Source File

SOURCE=..\InvestigationForward.h
# End Source File
# Begin Source File

SOURCE=..\InvestigationPerturbation.h
# End Source File
# Begin Source File

SOURCE=..\RegisterTypes_suAnalyses.h
# End Source File
# Begin Source File

SOURCE=..\suActuation.h
# End Source File
# Begin Source File

SOURCE=..\suActuatorGeneralizedForces.h
# End Source File
# Begin Source File

SOURCE=..\suActuatorPerturbation.h
# End Source File
# Begin Source File

SOURCE=..\suActuatorPerturbationIndependent.h
# End Source File
# Begin Source File

SOURCE=..\suAnalyses.h
# End Source File
# Begin Source File

SOURCE=..\suAnalysesDLL.h
# End Source File
# Begin Source File

SOURCE=..\suBodyIndAcc.h
# End Source File
# Begin Source File

SOURCE=..\suBodyIndAccCOM.h
# End Source File
# Begin Source File

SOURCE=..\suBodyIndPowers.h
# End Source File
# Begin Source File

SOURCE=..\suBodyKinematics.h
# End Source File
# Begin Source File

SOURCE=..\suBodyPointIndAcc.h
# End Source File
# Begin Source File

SOURCE=..\suContact.h
# End Source File
# Begin Source File

SOURCE=..\suDecomp.h
# End Source File
# Begin Source File

SOURCE=..\suDecompHard.h
# End Source File
# Begin Source File

SOURCE=..\suDecompHardUTWalk.h
# End Source File
# Begin Source File

SOURCE=..\suDecompHardUTWalkNoComp.h
# End Source File
# Begin Source File

SOURCE=..\suDecompHardUTWalkNoCompPrescribed.h
# End Source File
# Begin Source File

SOURCE=..\suDecompInteg.h
# End Source File
# Begin Source File

SOURCE=..\suDecompNoComp.h
# End Source File
# Begin Source File

SOURCE=..\suDecompTarget.h
# End Source File
# Begin Source File

SOURCE=..\suDecompTargetNoComp.h
# End Source File
# Begin Source File

SOURCE=..\suDecompTargetNoCompPrescribed.h
# End Source File
# Begin Source File

SOURCE=..\suDecompTaylor.h
# End Source File
# Begin Source File

SOURCE=..\suForceApplier.h
# End Source File
# Begin Source File

SOURCE=..\suGeneralizedForceApplier.h
# End Source File
# Begin Source File

SOURCE=..\suGeneralizedForcePerturbation.h
# End Source File
# Begin Source File

SOURCE=..\suGeneralizedForces.h
# End Source File
# Begin Source File

SOURCE=..\suIndAcc.h
# End Source File
# Begin Source File

SOURCE=..\suIndContactPowers.h
# End Source File
# Begin Source File

SOURCE=..\suKinematics.h
# End Source File
# Begin Source File

SOURCE=..\suLinearSpring.h
# End Source File
# Begin Source File

SOURCE=..\suPointKinematics.h
# End Source File
# Begin Source File

SOURCE=..\suTorqueApplier.h
# End Source File
# Begin Source File

SOURCE=..\suTorsionalSpring.h
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# End Group
# End Target
# End Project
