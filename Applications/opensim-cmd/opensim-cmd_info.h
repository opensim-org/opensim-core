#ifndef OPENSIM_CMD_INFO_H_
#define OPENSIM_CMD_INFO_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  opensim-cmd_info.h                         *
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

#include <iostream>

#include <docopt.h>
#include "parse_arguments.h"

static const char HELP_INFO[] =
R"(Show description of properties in an OpenSim class.

Usage:
  opensim-cmd [options]... info [<class> [<property>]]
  opensim-cmd info -h | --help

Options:
  -L <path>, --library <path>  Load a plugin.
  -o <level>, --log <level>  Logging level.

Description:
  If you do not supply any arguments, you get a list of all registered
  classes, including those from plugins.

  If you supply just <class>, you get a list of the properties in that
  class. If you supply <property> as well, you also get a description
  of that property. You can also get descriptions for classes from plugins.

Examples:
  opensim-cmd info
  opensim-cmd info PathActuator
  opensim-cmd info Model gravity
)";

int info(int argc, const char** argv) {

    using namespace OpenSim;

    std::map<std::string, docopt::value> args = OpenSim::parse_arguments(
            HELP_INFO, { argv + 1, argv + argc },
            true); // show help if requested

    // No arguments were provided.
    if (!args["<class>"]) {
        Object::PrintPropertyInfo(std::cout, "", false);
        return EXIT_SUCCESS;
    }

    // Class was provided.
    const std::string& className = args["<class>"].asString();
    const Object* object = Object::getDefaultInstanceOfType(className);
    if (object == nullptr) {
        throw Exception( "No registered class with name '" + className +
                "'. Did you intend to load a plugin?");
    }

    // Property was not provided.
    if (!args["<property>"]) {
        Object::PrintPropertyInfo(std::cout, className, false);
        return EXIT_SUCCESS;
    }

    // Both class and property were provided.
    const std::string& propName = args["<property>"].asString();

    const bool success = Object::PrintPropertyInfo(std::cout,
            className, propName, false);
    if (!success) {
        throw Exception("No property with name '" + propName +
                "' found in class '" + className + "'.");
    }
    return EXIT_SUCCESS;
}

#endif // OPENSIM_CMD_INFO_H_
