/* -------------------------------------------------------------------------- *
 *                        OpenSim:  DebugUtilities.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include "DebugUtilities.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <cassert>
#include <stdexcept>
#include <cstdlib>

namespace OpenSim {
namespace DebugUtilities {

void Fatal_Error(const char *msg, const char *function, const char *file, unsigned int line)
{
    std::ostringstream string_stream;
    string_stream << "Fatal Error: " << msg << " (function = " << function << ", file = " << file << ", line = " << line << ")";
    std::cerr << string_stream.str() << std::endl;
    throw std::runtime_error(string_stream.str());
    assert(false);
    exit(1);
}

/**
 * Basically a utility that lets me take a bash script that has lines such as
 * export VAR=value
 * and insert these environment variables into this executable's environment.
 * Mostly for testing/debugging applications.
 */
void AddEnvironmentVariablesFromFile(const std::string &aFileName)
{
    if(aFileName.empty()) return;
    std::ifstream input(aFileName.c_str());
    std::string line;
    // Take any line that starts with "export" and set the environment variable that follows
    while (getline(input,line)) {
        if(line.find("export") != std::string::npos) {
            std::string env=line.substr(7);
            std::cout << "Setting environment '" << env << "'" << std::endl;
#ifdef WIN32
            _putenv(env.c_str());
#else
            putenv(const_cast<char*>(env.c_str()));
#endif
        }
    }
}

}
}
