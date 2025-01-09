#ifndef OPENSIM_LOAD_OPENSIM_LIBRARY_H_
#define OPENSIM_LOAD_OPENSIM_LIBRARY_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  LoadOpenSimLibrary.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

/** Load an OpenSim (plugin) library, using a path to a library (relative or
 * absolute) but *without* the file extension (.dll, .so, .dylib). This method
 * will prefer a debug variant of the library if OpenSim was built in debug.
 *
 * In MATLAB/Python, we suggest you use LoadOpenSimLibraryExact instead of this
 * function. If you insist on using this variant, see the examples below.
 *
 * To load a plugin in MATLAB, use the following:
 * @code
 * import org.opensim.modeling.*;
 * opensimCommon.LoadOpenSimLibrary('<path>/osimMyPlugin');
 * @endcode
 * Do NOT use MATLAB's `loadlibrary()`.
 *
 * To load a plugin in Python, use the following:
 * @code{.py}
 * import opensim
 * opensim.LoadOpenSimLibrary('<path>/osimMyPlugin')
 * @endcode  */
OSIMCOMMON_API OPENSIM_PORTABLE_HMODULE WINAPI LoadOpenSimLibrary(
        const std::string &lpLibFileName, bool verbose);
/** Uses LoadOpenSimLibrary(const std::string&, bool), with verbosity. */
OSIMCOMMON_API void LoadOpenSimLibrary(const std::string &aLibraryName);
/** Load an OpenSim (plugin) library using the exact path specified. Therefore,
 * you must supply an exact path to the library (either relative or absolute),
 * including the file extension (.dll, .so, .dylib). The only change that may
 * be made to the path is to convert forward slashes to backslashes on Windows
 * (and vice versa on UNIX).
 *
 * To load a plugin in MATLAB, use the following:
 * @code
 * import org.opensim.modeling.*;
 * opensimCommon.LoadOpenSimLibraryExact('<path>/osimMyPlugin.dll');
 * @endcode
 * Do NOT use MATLAB's `loadlibrary()`.
 *
 * To load a plugin in Python, use the following:
 * @code{.py}
 * import opensim
 * opensim.LoadOpenSimLibraryExact('<path>/osimMyPlugin.dll')
 * @endcode
 *
 * @note If your (plugin) library depends on other libraries, make sure they
 * are available as well (e.g., by setting the appropriate values for
 * environment variables like `PATH` (Windows), `LD_LIBRARY_PATH` (Linux), and
 * `DYLD_LIBRARY_PATH` (macOS)).
 * 
 * @returns true if the library was successfully loaded; false otherwise. */
OSIMCOMMON_API bool LoadOpenSimLibraryExact(const std::string &exactPath,
                                            bool verbose = true);
/** Used to process legacy command line arguments that specify plugin libraries
 * to load. Internally uses LoadOpenSimLibrary(const std::string&, bool) with
 * verbosity. */
OSIMCOMMON_API void LoadOpenSimLibraries(int argc, char **argv);

}

#endif // OPENSIM_LOAD_OPENSIM_LIBRARY_H_
