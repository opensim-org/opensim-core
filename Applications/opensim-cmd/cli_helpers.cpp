/* -------------------------------------------------------------------------- *
 *                       OpenSim:  opensim-cmd.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ayman Habib, Chris Dembia                    *
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

#include "cli_helpers.h"

#include <cstring>

bool OpenSimCmd::is_arg(const char* c, const char* cmp) {
    return not strcmp(c, cmp);
}

bool OpenSimCmd::is_help_arg(const char* c) {
    return is_arg(c, "--help", "-help", "-h");
}

bool OpenSimCmd::starts_with(const char* pre, const char* s) {
    return strncmp(pre, s, strlen(pre)) == 0;
}

bool OpenSimCmd::extract_opt_arg(int& argc, const char**& argv, const char* pre, const char*& out) {
    const char* arg = argv[0];
    auto prelen = strlen(pre);
    auto arglen = strlen(arg);

    if (arglen < prelen) { return false; }

    // -O arg
    if (arg[prelen] == '\0') {
        if (argc < 2) { return false; }
        out = argv[1];
        argc--;
        argv++;
        return true;
    }

    // -O=arg
    if (arg[prelen] == '=') {
        if (arg[prelen + 1] == '\0') { return false; }
        out = arg + prelen + 1;
        return true;
    }

    // -Oarg (but not --optarg)
    if (not starts_with("--", pre)) {
        out = arg + prelen;
        return true;
    }

    return false;
}