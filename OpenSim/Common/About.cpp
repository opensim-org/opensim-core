/* -------------------------------------------------------------------------- *
 *                            OpenSim:  About.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

/**
 * Define the standard SimTK compliant "version" and "about" routines.
 */

#include "About.h"
#include <cstring>
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>

#define STR(var) #var
#define MAKE_VERSION_STRING(maj,min,build)  STR(maj.min.build)
#define MAKE_COPYRIGHT_STRING(y,a) \
    "Copyright (c) " STR(y) " Stanford University, " STR(a)
#define MAKE_STRING(a) STR(a)

#define GET_VERSION_STRING  \
    MAKE_VERSION_STRING(OPENSIM_COMMON_MAJOR_VERSION,  \
                        OPENSIM_COMMON_MINOR_VERSION,  \
                        OPENSIM_COMMON_BUILD_VERSION)

#define GET_COPYRIGHT_STRING \
    MAKE_COPYRIGHT_STRING(OPENSIM_COMMON_COPYRIGHT_YEARS, \
                          OPENSIM_COMMON_AUTHORS)

#define GET_AUTHORS_STRING \
    MAKE_STRING(OPENSIM_COMMON_AUTHORS)

#define GET_LIBRARY_STRING \
    MAKE_STRING(OPENSIM_COMMON_LIBRARY_NAME)

#define GET_TYPE_STRING \
    MAKE_STRING(OPENSIM_COMMON_TYPE)

#define GET_SYSTEM_INFO \
    MAKE_STRING(OSIM_SYS_INFO)

#define GET_COMPILER_INFO \
    MAKE_STRING(OSIM_COMPILER_INFO)

#define GET_OS_NAME \
    MAKE_STRING(OSIM_OS_NAME)

#define GET_OSIM_VERSION \
    MAKE_STRING(OSIM_VERSION)

#ifndef NDEBUG
    #define GET_DEBUG_STRING "debug"
#else
    #define GET_DEBUG_STRING "release"
#endif

using namespace std;

extern "C" {

void opensim_version_common(int* major, int* minor, int* build) {
    static const char* l = "OPENSIM library="   GET_LIBRARY_STRING;
    static const char* t = "OPENSIM type="      GET_TYPE_STRING;
    static const char* d = "OPENSIM debug="     GET_DEBUG_STRING;
    static const char* v = "OPENSIM version="   GET_VERSION_STRING;
    static const char* c = "OPENSIM copyright=" GET_COPYRIGHT_STRING;

    if (major) *major = OPENSIM_COMMON_MAJOR_VERSION;
    if (minor) *minor = OPENSIM_COMMON_MINOR_VERSION;
    if (build) *build = OPENSIM_COMMON_BUILD_VERSION;

    // Force statics to be present in the binary (Release mode otherwise 
    // optimizes them away).
    volatile int i=0;
    if (i) { // never true, but compiler doesn't know ...
        *major = *l + *t + *d + *v + *c;
    }
}

void opensim_about_common(const char* key, int maxlen, char* value) {
    if (maxlen <= 0 || value==0) return;
    value[0] = '\0'; // in case we don't find a match
    if (key==0) return;

    // downshift the key
    std::string skey(key);
    for (size_t i=0; i<skey.size(); ++i)
        skey[i] = tolower(skey[i]);

    const char* v = 0;
    if      (skey == "version")   v = GET_VERSION_STRING;
    else if (skey == "library")   v = GET_LIBRARY_STRING;
    else if (skey == "type")      v = GET_TYPE_STRING;
    else if (skey == "copyright") v = GET_COPYRIGHT_STRING;
    else if (skey == "authors")   v = GET_AUTHORS_STRING;
    else if (skey == "debug")     v = GET_DEBUG_STRING;

    if (v) {
        strncpy(value,v,maxlen-1);
        value[maxlen-1] = '\0'; // in case we ran out of room
    }
}

} // extern "C"

namespace OpenSim {

static const char* OpenSimVersion = GET_OSIM_VERSION;

std::string GetVersionAndDate() { 
    char buffer[256];
    sprintf(buffer,"version %s, build date %s %s",
            OpenSimVersion, __TIME__, __DATE__);
    return std::string(buffer);
}

std::string GetVersion() {
    return OpenSimVersion;
}

std::string GetOSInfoVerbose() {
    const char * str = GET_SYSTEM_INFO;
    return str;
}

std::string GetOSInfo() {
    const char * str = GET_OS_NAME;
    return str;
}

std::string GetCompilerVersion() {
    std::string os = GetOSInfo();
    std::string str = "(Unknown)";

    if( 0 == os.compare("Windows")) {
        const int MSVCVersion = atoi(GET_COMPILER_INFO);
        if( MSVCVersion >= 1910 ) {
            // With Visual Studio 2017, the versioning of the Visual C++
            // compiler became more fine-grained, so we can no longer use
            // a switch statement.
            // Also, Visual Studio 2017 decouples the Visual Studio IDE
            // from the C++ toolset (compiler), so providing the IDE year 
            // does not indicate the compiler version (it may be possible
            // to use the Visual Studio 2019 IDE, or whatever is next, 
            // with the same C++ toolset that came with Visual Studio 2017.
            // Therefore, we no longer provide the Visual Studio year.
            // https://blogs.msdn.microsoft.com/vcblog/2016/10/05/visual-c-compiler-version/
            // https://en.wikipedia.org/wiki/Microsoft_Visual_C%2B%2B
            if (1910 <= MSVCVersion && MSVCVersion < 2000) {
                str = "Microsoft Visual C++ 14.1";
            }
            str += " (MSC_VER " + std::to_string(MSVCVersion) + ")";
        } else {
            switch( MSVCVersion ) {
                case 1900:
                    str = "Visual Studio 2015";
                    break;
                case 1800:
                    str = "Visual Studio 2013";
                    break;
                case 1700:
                    str = "Visual Studio 2011";
                    break;
                case 1600:
                    str = "Visual Studio 2010";
                    break;
                case 1500:
                    str = "Visual Studio 2008";
                    break;
                case 1400:
                    str = "Visual Studio 2005";
                    break;
                case 1310:
                    str = "Visual Studio 2003";
                    break;
                case 1300:
                    str = "Visual Studio 2002";
                    break;
            }
        }
    } else if( 0 == os.compare("Darwin")) {
        str = "Mac OS X :";
        str += GET_COMPILER_INFO;
    } else if( 0 == os.compare("Linux")){
        str = "Linux :";
        str = GET_COMPILER_INFO;
    } else {
        str = GET_COMPILER_INFO;
    }

    return str;
}

}
