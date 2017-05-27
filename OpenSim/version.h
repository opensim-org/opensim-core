#ifndef _version_h_
#define _version_h_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  version.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#if defined(__cplusplus) || defined(SWIG)
#include <string>
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>

#define STR(var) #var
#define MAKE_STRING(a) STR(a)
#define GET_SYSTEM_INFO \
    MAKE_STRING(OSIM_SYS_INFO)

#define GET_COMPILER_INFO \
    MAKE_STRING(OSIM_COMPILER_INFO)

#define GET_OS_NAME \
    MAKE_STRING(OSIM_OS_NAME)

#define GET_OSIM_VERSION \
    MAKE_STRING(OSIM_VERSION)

namespace OpenSim {
#endif

    static const char *OpenSimVersion = GET_OSIM_VERSION;

#if defined(__cplusplus) || defined(SWIG)
    inline std::string GetVersionAndDate() { 
        char buffer[256];
        sprintf(buffer,"version %s, build date %s %s", OpenSimVersion, __TIME__, __DATE__);
        return std::string(buffer);
    }

    inline std::string GetVersion() {
        return OpenSimVersion;
    }

    inline std::string GetOSInfoVerbose() {
        const char * str = GET_SYSTEM_INFO;
        return str;
    }
    inline std::string GetOSInfo() {
        const char * str = GET_OS_NAME;
        return str;
    }
    inline std::string GetCompilerVersion() {
        std::string os = GetOSInfo();
        std::string str;

        if( 0 == os.compare("Windows")) {
            switch( atoi(GET_COMPILER_INFO) ) {
            case 1910:
                str = "Visual Studio 2017";
                break;
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
#endif

#endif
