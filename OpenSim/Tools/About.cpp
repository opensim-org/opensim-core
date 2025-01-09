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


#include "osimToolsDLL.h"

#include <string>
#include <cstring>
#include <cctype>


#define STR(var) #var
#define MAKE_VERSION_STRING(maj,min,build)  STR(maj.min.build)
#define MAKE_COPYRIGHT_STRING(y,a) \
    "Copyright (c) " STR(y) " Stanford University, " STR(a)
#define MAKE_STRING(a) STR(a)

#define GET_VERSION_STRING  \
    MAKE_VERSION_STRING(OPENSIM_TOOLS_MAJOR_VERSION,  \
                        OPENSIM_TOOLS_MINOR_VERSION,  \
                        OPENSIM_TOOLS_BUILD_VERSION)

#define GET_COPYRIGHT_STRING \
    MAKE_COPYRIGHT_STRING(OPENSIM_TOOLS_COPYRIGHT_YEARS, \
                          OPENSIM_TOOLS_AUTHORS)

#define GET_AUTHORS_STRING \
    MAKE_STRING(OPENSIM_TOOLS_AUTHORS)

#define GET_LIBRARY_STRING \
    MAKE_STRING(OPENSIM_TOOLS_LIBRARY_NAME)

#define GET_TYPE_STRING \
    MAKE_STRING(OPENSIM_TOOLS_TYPE)

#ifndef NDEBUG
    #define GET_DEBUG_STRING "debug"
#else
    #define GET_DEBUG_STRING "release"
#endif


using namespace std;


extern "C" {

void opensim_version_tools(int* major, int* minor, int* build) {
    static const char* l = "OPENSIM library="   GET_LIBRARY_STRING;
    static const char* t = "OPENSIM type="      GET_TYPE_STRING;
    static const char* d = "OPENSIM debug="     GET_DEBUG_STRING;
    static const char* v = "OPENSIM version="   GET_VERSION_STRING;
    static const char* c = "OPENSIM copyright=" GET_COPYRIGHT_STRING;

    if (major) *major = OPENSIM_TOOLS_MAJOR_VERSION;
    if (minor) *minor = OPENSIM_TOOLS_MINOR_VERSION;
    if (build) *build = OPENSIM_TOOLS_BUILD_VERSION;

    // Force statics to be present in the binary (Release mode otherwise 
    // optimizes them away).
    volatile int i=0;
    if (i) { // never true, but compiler doesn't know ...
        *major = *l + *t + *d + *v + *c;
    }
}

void opensim_about_tools(const char* key, int maxlen, char* value) {
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

}
