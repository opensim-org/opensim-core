#ifndef _LoadOpenSimLibrary_h_
#define _LoadOpenSimLibrary_h_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  LoadOpenSimLibrary.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ayman Habib                                  *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <string>
#include "osimCommonDLL.h"

#ifndef _WIN32
#define OPENSIM_PORTABLE_HMODULE void *
#define OPENSIM_PORTABLE_HINSTANCE void *
#define WINAPI
#include <dlfcn.h>
#define GetProcAddress(handle, proc) dlsym(handle, proc)
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
