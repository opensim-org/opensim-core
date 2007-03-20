#ifndef _LoadOpenSimLibrary_h_
#define _LoadOpenSimLibrary_h_
// LoadOpenSimLibrary.h
// author: Frank C. Anderson, Ayman Habib
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include "osimCommonDLL.h"

#ifdef __linux__
#define OPENSIM_PORTABLE_HMODULE void *
#define OPENSIM_PORTABLE_HINSTANCE void *
#define WINAPI
#else
#define OPENSIM_PORTABLE_HMODULE HMODULE
#define OPENSIM_PORTABLE_HINSTANCE HINSTANCE
#endif

namespace OpenSim
{

OSIMCOMMON_API OPENSIM_PORTABLE_HMODULE WINAPI LoadOpenSimLibrary(const std::string &lpLibFileName, bool verbose);
OSIMCOMMON_API void LoadOpenSimLibrary(const std::string &aLibraryName);
OSIMCOMMON_API void LoadOpenSimLibraries(int argc,char **argv);

}

#endif // __LoadOpenSimLibrary_h__
