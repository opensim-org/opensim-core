/* -------------------------------------------------------------------------- *
 *                      OpenSim:  LoadOpenSimLibrary.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "LoadOpenSimLibrary.h"

#include "IO.h"
#include "Logger.h"
#include <iostream>

using namespace OpenSim;
using namespace std;

#ifndef _WIN32
// For Linux compatibility is to remap
// LoadLibrary/GetProcAddress to dlopen/dlsym.
static void *LoadLibrary(const char* name) {
    return dlopen(name, RTLD_LAZY | RTLD_GLOBAL);
}

// We want to transparently support adding the "lib" prefix to library names
// when loading them.
static void *LoadLibrary(const std::string &name, std::string &actualNameLoaded) {
    actualNameLoaded = name;
    void *lib = LoadLibrary(name.c_str());
    if(!lib) {
        std::string libName = OpenSim::IO::GetFileNameFromURI(name);
        if(libName.size()<3 || libName.substr(0,3)!="lib") { // if it doesn't already have lib prefix
            libName = OpenSim::IO::getParentDirectory(name) + "lib" + libName;
            actualNameLoaded = libName;
            //std::cout << "Loading " << name << " failed, trying " << libName << " (for Linux compatibility)" << std::endl;
            lib = LoadLibrary(libName.c_str());
        }
    }
    return lib;
}
#define LoadLibraryError() { char* err=dlerror(); if(err) log_error("dlerror: {}", err); }

#else

HINSTANCE LoadLibrary(const std::string &name, std::string &actualNameLoaded) {
    actualNameLoaded = name;
    return LoadLibrary(name.c_str());
}
#define LoadLibraryError()

#endif

//_____________________________________________________________________________
/**
 * A wrapper around Window's LoadLibrary that implements library naming
 * convention and loading policy on windows which follows:
 * If you're loading osimSimulation_D and other libraries that do not have a
 * trailing _D an _D is appended to the library file name.  If loading of that
 * fails, we revert to using the non _D file instead, if that fails we give
 * error and return 0. A reciprocal treatment for release libraries is
 * implemented. I tried to keep this function in the same file to try
 * to localize platform specific code. -Ayman
 *
 * @param lpLibFileName Name of the library without either the .lib or .dll
 * extension.
 * @return Pointer to the loaded library, NULL on error.
 */

OSIMCOMMON_API
OPENSIM_PORTABLE_HMODULE
WINAPI
OpenSim::LoadOpenSimLibrary(const std::string &lpLibFileName, bool verbose)
{
    string libraryExtension;
#ifdef __linux__
    libraryExtension = ".so";
#elif defined(__APPLE__)
    libraryExtension = ".dylib";
#endif
    string fixedLibFileName = IO::FixSlashesInFilePath(lpLibFileName);
    string actualLibFileName = fixedLibFileName + libraryExtension;
    static const string debugSuffix = "_d";
    bool hasDebugSuffix = (IO::GetSuffix(fixedLibFileName,(int)debugSuffix.size())==debugSuffix);
    string actualNameLoaded;

    OPENSIM_PORTABLE_HINSTANCE libraryHandle = NULL;

    // If we're in debug mode and a release library is specified, or we're in release
    // mode and a debug library is specified, we'll first try loading the debug library,
    // and then try loading the release library.
    bool tryDebugThenRelease = false;
#ifndef NDEBUG
    if(!hasDebugSuffix) {
        if(verbose) log_info("Will try loading debug library first");
        tryDebugThenRelease = true;
    }
#else
    if(hasDebugSuffix) {
        if(verbose) log_warn("Trying to load a debug library into release osimSimulation");
        tryDebugThenRelease = true;
    }
#endif

    if(tryDebugThenRelease) {
        if(hasDebugSuffix) IO::RemoveSuffix(fixedLibFileName,(int)debugSuffix.size());
        string debugLibFileName = fixedLibFileName + debugSuffix + libraryExtension;
        string releaseLibFileName = fixedLibFileName + libraryExtension;
        if ((libraryHandle = LoadLibrary(debugLibFileName,actualNameLoaded))) {
            if(verbose) log_info("Loaded library {}", actualNameLoaded);
        } else {
            LoadLibraryError();
            if(verbose) {
                log_error("Loading of debug library {} failed. Trying {}.",
                        debugLibFileName, releaseLibFileName);
            }
            if ((libraryHandle = LoadLibrary(releaseLibFileName,actualNameLoaded))) {
                if(verbose) log_info("Loaded library {}", actualNameLoaded);
            } else {
                LoadLibraryError();
                if(verbose)
                    log_error("Failed to load either debug or release "
                              "library {}.", releaseLibFileName);
            }
        }
    } else {
        if ((libraryHandle = LoadLibrary(actualLibFileName,actualNameLoaded))) {
            if(verbose) log_info("Loaded library {}", actualNameLoaded);
        } else {
            LoadLibraryError();
            if (verbose)
                log_error("Failed to load library {}", actualLibFileName);
        }
    }

    return libraryHandle;
}

OSIMCOMMON_API void
OpenSim::LoadOpenSimLibrary(const std::string &aLibraryName)
{
    LoadOpenSimLibrary(aLibraryName, true);
}

OSIMCOMMON_API bool
OpenSim::LoadOpenSimLibraryExact(const std::string& exactPath,
                                 bool verbose) {
    const auto fixedPath = IO::FixSlashesInFilePath(exactPath);
    OPENSIM_PORTABLE_HINSTANCE library = LoadLibrary(fixedPath.c_str());
    if (library) {
        if (verbose) {
            log_info("Loaded library {}", fixedPath);
        }
        return true;
    } else {
        if (verbose) {
            log_error("Failed to load library {}", fixedPath);
        }
        return false;
    }
}

//_____________________________________________________________________________
/**
 * A function for loading libraries specified in a command line.
 * LoadOpenSimLibrary() is used to load each library.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @see LoadOpenSimLibrary()
 */

OSIMCOMMON_API void 
OpenSim::LoadOpenSimLibraries(int argc,char **argv)
{
    int i;
    string option;
    OPENSIM_PORTABLE_HINSTANCE library;
    for(i=0;i<argc;i++) {
        if(argv[i][0]!='-') continue;
        option = argv[i];
        if((i+1)>=argc) break;  // no more arguments.
        if((option=="-Library")||(option=="-L")) {
            string libraryName = argv[i+1];
            library = LoadOpenSimLibrary(libraryName.c_str(), true);
            if(library==NULL) {
                log_error("Library {} could not be loaded.", libraryName);
            } else {
                i++;
            }
        }
    }
}
