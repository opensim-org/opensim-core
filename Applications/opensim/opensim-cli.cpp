/* -------------------------------------------------------------------------- *
 *                       OpenSim:  opensim.cpp                                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
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

#include "opensim_run_tool.h"
#include "opensim_print_xml.h"
#include "opensim_info.h"
#include "opensim_update_file.h"

#include <iostream>

#include <docopt.h>

#include <OpenSim/OpenSim.h>
#include <OpenSim/version.h>

static const char HELP[] = 
R"(OpenSim: musculoskeletal modeling and simulation.

Usage:
  opensim [--library=<path>]... <command> [<args>...]
  opensim -h | --help
  opensim -V | --version

Options:
  -L <path>, --library <path>  Load a plugin before executing the requested
                 command. The <path> to the library can be absolute or relative
                 to the current directory, Do NOT provide the plugin's
                 extension (e.g., .dll, .so, .dylib). You can load multiple
                 plugins by repeating this option.
  -h, --help     Show this help description.
  -V, --version  Show the version number.

Available commands:
  run-tool     Run a Tool (IK, CMC, ...) from an XML setup file.
  print-xml    Print a template XML file for a Tool or class.
  info         Show description of properties in an OpenSim class.
  update-file  Update an .xml file (.osim or setup) to this version's format.

  Pass -h or --help to any of these commands to learn how to use them.
)";
//  -L <plugin>, --library <plugin>  Load a plugin before executing the provided command.

int main(int argc, const char** argv) {

    using namespace OpenSim;

    try {

    // Register the available commands.
    // --------------------------------
    std::map<std::string, std::function<int (int, const char**)>> commands;

    commands["print-xml"] = print_xml;
    commands["run-tool"] = run_tool;
    commands["info"] = info;
    commands["update-file"] = update_file;

    // If no arguments are provided; just print the help text.
    // -------------------------------------------------------
    if (argc == 1) {
        std::cout << HELP << std::endl;
        return EXIT_SUCCESS;
    }

    // Parse the arguments.
    // --------------------
    std::map<std::string, docopt::value> args = docopt::docopt(
            HELP, { argv + 1, argv + argc },
            true, // show help if requested
            "OpenSim " + GetVersionAndDate(),
            true); // take options first; necessary for nested commands.

    // Load the specified libraries.
    // -----------------------------
    if (args["--library"]) {
        for (const auto& plugin : args["--library"].asStringList()) {
            const auto& lib = LoadOpenSimLibrary(plugin, true);
            // Exit if we couldn't load the library.
            if (!lib) return EXIT_FAILURE;
        }
    }

    // Did the user provide a valid command?
    // -------------------------------------
    if (!args["<command>"]) {
        std::cout << "No command provided." << std::endl;
        return EXIT_FAILURE;
    }

    // The user provided a command.
    const auto& command = args["<command>"].asString();

    if (commands.count(command) == 0) {
        std::cout << "'" << command << "' is not an opensim command. "
            << "See 'opensim --help'." << std::endl;
        return EXIT_FAILURE;
    }

    // Dispatch to the requested command.
    // ----------------------------------
    // Each command returns whether or not it succeeded.
    return commands.at(command)(argc, argv);


    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // We shouldn't ever get here, so we can consider this a failure.
    return EXIT_FAILURE;
}



