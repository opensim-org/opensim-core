/* -------------------------------------------------------------------------- *
 * OpenSim Moco: About.cpp                                                    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
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

#include <stdio.h>

#include <OpenSim/Common/Logger.h>

#define STR(var) #var
#define MAKE_VERSION_STRING(maj, min, build) STR(maj.min.build)
#define MAKE_STRING(a) STR(a)

#define GET_OPENSIM_MOCO_VERSION_STRING MAKE_STRING(OPENSIM_MOCO_VERSION)

#ifndef NDEBUG
#    define GET_DEBUG_STRING "debug"
#else
#    define GET_DEBUG_STRING "release"
#endif

using namespace std;

namespace OpenSim {

static const char* OpenSimMocoVersion = GET_OPENSIM_MOCO_VERSION_STRING;

std::string GetMocoVersionAndDate() {
    return fmt::format("version {}, build date {} {}", OpenSimMocoVersion,
            __TIME__, __DATE__);
}

std::string GetMocoVersion() { return OpenSimMocoVersion; }

} // namespace OpenSim
