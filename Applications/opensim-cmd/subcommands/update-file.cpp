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

#include "update-file.h"

#include "Applications/opensim-cmd/cli_helpers.h"

#include <OpenSim/OpenSim.h>

static const char update_file_help[] =
        R"(Update an .osim, .xml (e.g., setup) or .sto file to this version's format.

Description:
  In an OpenSim XML file, the XML file format version appears as
  the "Version" attribute of the "OpenSimDocument" element. The XML file format
  version number is generally not the same as the OpenSim software version
  number.

Examples:
  opensim-cmd update-file lowerlimb_v3.3.osim lowerlimb_updated.osim
  opensim-cmd update-file RRA_taskset_v3.3.xml RRA_taskset_updated.osim
  opensim-cmd update-file data_v3.3.sto data_updated.sto
)";

int OpenSimCmd::update_file(int argc, const char** argv) {
    const char* cmdname = argv[0];
    argc--;
    argv++;

    for (; argc > 0; argc--, argv++) {
        const char* arg = argv[0];

        if (*arg != '-') {
            break;
        } else if (OpenSimCmd::is_help_arg(arg)) {
            std::cout << update_file_help << std::endl;
            return EXIT_SUCCESS;
        }
    }

    if (argc != 2) {
        std::cerr << "Arguments did not match expected patterns" << std::endl;
        return EXIT_FAILURE;
    }

    const std::string inputFile{argv[0]};
    const std::string outputFile{argv[1]};

    // Grab the file extension.
    std::string::size_type extSep = inputFile.rfind('.');
    if (extSep == std::string::npos) {
        std::cerr << "Input file '"
                  << inputFile
                  << "' does not have an extension."
                  << std::endl;
        return EXIT_FAILURE;
    }
    std::string extension = inputFile.substr(extSep);

    // .osim or .xml file.
    if (extension == ".osim" or extension == ".xml") {
        std::cout << "Loading input file '" << inputFile << "'." << std::endl;
        const auto* obj = OpenSim::Object::makeObjectFromFile(inputFile);

        if (obj == nullptr) {
            std::cerr << "Could not make object from file '" << inputFile
                      << "'.\n"
                      << "Did you intend to load a plugin (with --library)?";
            return EXIT_FAILURE;
        }

        std::cout << "Printing updated file to '" << outputFile << "'." << std::endl;
        obj->print(outputFile);
        return EXIT_SUCCESS;
    }

    // .sto file.
    if (extension == ".sto") {
        std::cout << "Loading input file '" << inputFile << "'." << std::endl;
        OpenSim::Storage stg{inputFile};
        std::cout << "Printing updated file to '" << outputFile << "'." << std::endl;
        stg.print(outputFile);
        return EXIT_SUCCESS;
    }

    std::cerr  << "Input file '"  << inputFile
               << "' has an unrecognized extension." << std::endl;
    return EXIT_FAILURE;
}

