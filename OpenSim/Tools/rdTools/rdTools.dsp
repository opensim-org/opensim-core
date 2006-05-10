# Microsoft Developer Studio Project File - Name="rdTools" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Dynamic-Link Library" 0x0102

CFG=rdTools - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "rdTools.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "rdTools.mak" CFG="rdTools - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "rdTools - Win32 Release" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE "rdTools - Win32 Debug" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
F90=df.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "rdTools - Win32 Release"

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
# ADD F90 /compile_only /debug:none /dll /nologo /threads /warn:nofileopt
# ADD BASE CPP /nologo /MT /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /D "RDTOOLS_EXPORTS" /YX /FD /c
# ADD CPP /nologo /MD /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /D "RDTOOLS_EXPORTS" /FD /c
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
# ADD LINK32 xerces-c_2.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /dll /machine:I386
# Begin Special Build Tool
SOURCE="$(InputPath)"
PostBuild_Desc=Updating libraries...
PostBuild_Cmds=$(RDI_INSTALL)\Bin\cpl
# End Special Build Tool

!ELSEIF  "$(CFG)" == "rdTools - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "rdTools___Win32_Debug"
# PROP BASE Intermediate_Dir "rdTools___Win32_Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE F90 /check:bounds /compile_only /debug:full /dll /nologo /traceback /warn:argument_checking /warn:nofileopt
# ADD F90 /check:bounds /compile_only /dbglibs /debug:full /dll /nologo /threads /traceback /warn:argument_checking /warn:nofileopt
# ADD BASE CPP /nologo /MTd /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_MBCS" /D "_USRDLL" /D "RDTOOLS_EXPORTS" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "PLATFORM_WIN32" /D "_MBCS" /D "_USRDLL" /D "RDTOOLS_EXPORTS" /FR /FD /GZ /c
# SUBTRACT CPP /YX
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# SUBTRACT BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /dll /debug /machine:I386 /pdbtype:sept
# ADD LINK32 xerces-c_2D.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /dll /debug /machine:I386 /out:"Debug/rdTools_D.dll" /pdbtype:sept
# Begin Special Build Tool
SOURCE="$(InputPath)"
PostBuild_Desc=Updating libraries...
PostBuild_Cmds=$(RDI_INSTALL)\Bin\cpl D
# End Special Build Tool

!ENDIF 

# Begin Target

# Name "rdTools - Win32 Release"
# Name "rdTools - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat;f90;for;f;fpp"
# Begin Source File

SOURCE=..\gcvspl.c
# End Source File
# Begin Source File

SOURCE=..\rdBodyConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\rdException.cpp
# End Source File
# Begin Source File

SOURCE=..\rdFunction.cpp
# End Source File
# Begin Source File

SOURCE=..\rdFunctionSet.cpp
# End Source File
# Begin Source File

SOURCE=..\rdGCVSpline.cpp
# End Source File
# Begin Source File

SOURCE=..\rdGCVSplineSet.cpp
# End Source File
# Begin Source File

SOURCE=..\rdIO.cpp
# End Source File
# Begin Source File

SOURCE=..\rdLine.cpp
# End Source File
# Begin Source File

SOURCE=..\rdMaterial.cpp
# End Source File
# Begin Source File

SOURCE=..\rdMaterialManager.cpp
# End Source File
# Begin Source File

SOURCE=..\rdMaterialSet.cpp
# End Source File
# Begin Source File

SOURCE=..\rdMath.cpp
# End Source File
# Begin Source File

SOURCE=..\rdMemory.cpp
# End Source File
# Begin Source File

SOURCE=..\rdMtx.cpp
# End Source File
# Begin Source File

SOURCE=..\rdObject.cpp
# End Source File
# Begin Source File

SOURCE=..\rdObservable.cpp
# End Source File
# Begin Source File

SOURCE=..\rdPlane.cpp
# End Source File
# Begin Source File

SOURCE=..\rdPointConstraint.cpp
# End Source File
# Begin Source File

SOURCE=..\rdProperty.cpp
# End Source File
# Begin Source File

SOURCE=..\rdPropertyBool.cpp
# End Source File
# Begin Source File

SOURCE=..\rdPropertyBoolArray.cpp
# End Source File
# Begin Source File

SOURCE=..\rdPropertyDbl.cpp
# End Source File
# Begin Source File

SOURCE=..\rdPropertyDblArray.cpp
# End Source File
# Begin Source File

SOURCE=..\rdPropertyInt.cpp
# End Source File
# Begin Source File

SOURCE=..\rdPropertyIntArray.cpp
# End Source File
# Begin Source File

SOURCE=..\rdPropertyObj.cpp
# End Source File
# Begin Source File

SOURCE=..\rdPropertyObjArray.cpp
# End Source File
# Begin Source File

SOURCE=..\rdPropertySet.cpp
# End Source File
# Begin Source File

SOURCE=..\rdPropertyStr.cpp
# End Source File
# Begin Source File

SOURCE=..\rdPropertyStrArray.cpp
# End Source File
# Begin Source File

SOURCE=..\rdSignal.cpp
# End Source File
# Begin Source File

SOURCE=..\rdSIMMUtilities.cpp
# End Source File
# Begin Source File

SOURCE=..\rdSpline.cpp
# End Source File
# Begin Source File

SOURCE=..\rdStateVector.cpp
# End Source File
# Begin Source File

SOURCE=..\rdStorage.cpp
# End Source File
# Begin Source File

SOURCE=..\rdTools.cpp
# End Source File
# Begin Source File

SOURCE=..\rdToolsDLL.cpp
# End Source File
# Begin Source File

SOURCE=..\rdToolsTemplates.cpp
# End Source File
# Begin Source File

SOURCE=..\rdTransform.cpp
# End Source File
# Begin Source File

SOURCE=..\rdVectorFunction.cpp
# End Source File
# Begin Source File

SOURCE=..\rdVectorGCVSplineR1R3.cpp
# End Source File
# Begin Source File

SOURCE=..\rdVisibleObject.cpp
# End Source File
# Begin Source File

SOURCE=..\rdVisibleProperties.cpp
# End Source File
# Begin Source File

SOURCE=..\rdXMLDocument.cpp
# End Source File
# Begin Source File

SOURCE=..\rdXMLNode.cpp
# End Source File
# Begin Source File

SOURCE=..\RegisterTypes_rdTools.cpp
# End Source File
# Begin Source File

SOURCE=..\RootSolver.cpp
# End Source File
# Begin Source File

SOURCE=..\simmNatCubicSpline.cpp
# End Source File
# Begin Source File

SOURCE=..\suScale.cpp
# End Source File
# Begin Source File

SOURCE=..\suScaleSet.cpp
# End Source File
# Begin Source File

SOURCE=..\VectorFunctionUncoupledNxN.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl;fi;fd"
# Begin Source File

SOURCE=..\gcvspl.h
# End Source File
# Begin Source File

SOURCE=..\rdArray.h
# End Source File
# Begin Source File

SOURCE=..\rdArrayPtrs.h
# End Source File
# Begin Source File

SOURCE=..\rdBodyConstraint.h
# End Source File
# Begin Source File

SOURCE=..\rdException.h
# End Source File
# Begin Source File

SOURCE=..\rdFunction.h
# End Source File
# Begin Source File

SOURCE=..\rdFunctionSet.h
# End Source File
# Begin Source File

SOURCE=..\rdGCVSpline.h
# End Source File
# Begin Source File

SOURCE=..\rdGCVSplineSet.h
# End Source File
# Begin Source File

SOURCE=..\rdIO.h
# End Source File
# Begin Source File

SOURCE=..\rdLine.h
# End Source File
# Begin Source File

SOURCE=..\rdMaterial.h
# End Source File
# Begin Source File

SOURCE=..\rdMaterialManager.h
# End Source File
# Begin Source File

SOURCE=..\rdMaterialSet.h
# End Source File
# Begin Source File

SOURCE=..\rdMath.h
# End Source File
# Begin Source File

SOURCE=..\rdMemory.h
# End Source File
# Begin Source File

SOURCE=..\rdMtx.h
# End Source File
# Begin Source File

SOURCE=..\rdNamedValueArray.h
# End Source File
# Begin Source File

SOURCE=..\rdObject.h
# End Source File
# Begin Source File

SOURCE=..\rdObservable.h
# End Source File
# Begin Source File

SOURCE=..\rdPlane.h
# End Source File
# Begin Source File

SOURCE=..\rdPointConstraint.h
# End Source File
# Begin Source File

SOURCE=..\rdProperty.h
# End Source File
# Begin Source File

SOURCE=..\rdPropertyBool.h
# End Source File
# Begin Source File

SOURCE=..\rdPropertyBoolArray.h
# End Source File
# Begin Source File

SOURCE=..\rdPropertyDbl.h
# End Source File
# Begin Source File

SOURCE=..\rdPropertyDblArray.h
# End Source File
# Begin Source File

SOURCE=..\rdPropertyInt.h
# End Source File
# Begin Source File

SOURCE=..\rdPropertyIntArray.h
# End Source File
# Begin Source File

SOURCE=..\rdPropertyObj.h
# End Source File
# Begin Source File

SOURCE=..\rdPropertyObjArray.h
# End Source File
# Begin Source File

SOURCE=..\rdPropertySet.h
# End Source File
# Begin Source File

SOURCE=..\rdPropertyStr.h
# End Source File
# Begin Source File

SOURCE=..\rdPropertyStrArray.h
# End Source File
# Begin Source File

SOURCE=..\rdSet.h
# End Source File
# Begin Source File

SOURCE=..\rdSignal.h
# End Source File
# Begin Source File

SOURCE=..\rdSIMMUtilities.h
# End Source File
# Begin Source File

SOURCE=..\rdSpline.h
# End Source File
# Begin Source File

SOURCE=..\rdStateVector.h
# End Source File
# Begin Source File

SOURCE=..\rdStorage.h
# End Source File
# Begin Source File

SOURCE=..\rdTools.h
# End Source File
# Begin Source File

SOURCE=..\rdToolsDLL.h
# End Source File
# Begin Source File

SOURCE=..\rdToolsTemplates.h
# End Source File
# Begin Source File

SOURCE=..\rdTransform.h
# End Source File
# Begin Source File

SOURCE=..\rdVectorFunction.h
# End Source File
# Begin Source File

SOURCE=..\rdVectorGCVSplineR1R3.h
# End Source File
# Begin Source File

SOURCE=..\rdVisibleObject.h
# End Source File
# Begin Source File

SOURCE=..\rdVisibleProperties.h
# End Source File
# Begin Source File

SOURCE=..\rdXMLDocument.h
# End Source File
# Begin Source File

SOURCE=..\rdXMLNode.h
# End Source File
# Begin Source File

SOURCE=..\RegisterTypes_rdTools.h
# End Source File
# Begin Source File

SOURCE=..\RootSolver.h
# End Source File
# Begin Source File

SOURCE=..\simmNatCubicSpline.h
# End Source File
# Begin Source File

SOURCE=..\suScale.h
# End Source File
# Begin Source File

SOURCE=..\suScaleSet.h
# End Source File
# Begin Source File

SOURCE=..\VectorFunctionUncoupledNxN.h
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# End Group
# End Target
# End Project
