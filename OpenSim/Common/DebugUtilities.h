#ifndef _DebugUtilities_h_
#define _DebugUtilities_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  DebugUtilities.h                         *
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
#include "osimCommonDLL.h"
#include <string>

#ifdef _WIN32
#define OPENSIM_NORETURN(declaration) __declspec(noreturn) declaration
#else
#define OPENSIM_NORETURN(declaration) declaration __attribute__ ((noreturn))
#endif

#define OPENSIM_ERROR_IF_NOT_OVERRIDDEN() \
    OpenSim::DebugUtilities::Fatal_Error("Function is not overridden", __FUNCTION__,__FILE__,__LINE__);

#define OPENSIM_FUNCTION_NOT_IMPLEMENTED() \
    OpenSim::DebugUtilities::Fatal_Error("Function is not (fully) implemented", __FUNCTION__,__FILE__,__LINE__);

namespace OpenSim {
namespace DebugUtilities {

OPENSIM_NORETURN(OSIMCOMMON_API void Fatal_Error(const char *msg, const char *function, const char *file, unsigned int line));

OSIMCOMMON_API void AddEnvironmentVariablesFromFile(const std::string &aFileName);

}
}

#endif
