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

#include "info.h"

#include "Applications/opensim-cmd/cli_helpers.h"

#include <OpenSim/OpenSim.h>

#include <iostream>

static const char info_help[] =
        R"(Show description of properties in an OpenSim class.

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

int OpenSimCmd::info(int argc, const char** argv) {
    // skip command-name arg.
    argc--;
    argv++;

    for (; argc > 0; argc--, argv++) {
        const char* arg = argv[0];

        if (arg[0] != '-') {
            break;
        } else if (OpenSimCmd::is_help_arg(arg)) {
            std::cout << info_help << std::endl;
            return EXIT_SUCCESS;
        }
    }

    // No arguments were provided.
    if (argc == 0) {
        OpenSim::Object::PrintPropertyInfo(
                std::cout,
                "",
                false);
        return EXIT_SUCCESS;
    }

    const std::string className{argv[0]};

    const OpenSim::Object* object = OpenSim::Object::getDefaultInstanceOfType(className);
    if (object == nullptr) {
        std::cerr << "No registered class with name '"
                  << className
                  << "'. Did you intend to load a plugin?"
                  << std::endl;
        return EXIT_FAILURE;
    }

    // Property was not provided.
    if (argc == 1) {
        OpenSim::Object::PrintPropertyInfo(std::cout, className, false);
        return EXIT_SUCCESS;
    }

    const char* propName = argv[1];

    if (OpenSim::Object::PrintPropertyInfo(std::cout, className, propName, false)) {
        return EXIT_SUCCESS;
    } else {
        std::cerr << "No property with name '"
                  << propName
                  << "' found in class '"
                  << className
                  << "'."
                  << std::endl;
        return EXIT_FAILURE;
    }
}