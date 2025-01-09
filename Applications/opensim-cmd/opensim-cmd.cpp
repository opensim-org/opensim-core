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

#include "opensim-cmd_info.h"
#include "opensim-cmd_print-xml.h"
#include "opensim-cmd_run-tool.h"
#include "opensim-cmd_update-file.h"
#include "opensim-cmd_viz.h"
#include "parse_arguments.h"
#include <docopt.h>
#include <iostream>
#include <OpenSim/OpenSim.h>
#include <OpenSim/version.h>

// The initial descriptions of the options should not exceed one line.
// This is to avoid regex errors with MSVC, where parsing HELP with
// MSVC's regex library causes a stack error if the description for an
// option is too long.

static const char HELP[] =
R"(OpenSim: musculoskeletal modeling and simulation.

Usage:
  opensim-cmd [--library=<path>]... [--log=<level>] <command> [<args>...]
  opensim-cmd -h | --help
  opensim-cmd -V | --version

Options:
  -L <path>, --library <path>  Load a plugin.
  -o <level>, --log <level>  Logging level.
  -h, --help     Show this help description.
  -V, --version  Show the version number.

Available commands:
  run-tool     Run a tool (e.g., Inverse Kinematics) from an XML setup file.
  print-xml    Print a template XML file for a Tool or class.
  info         Show description of properties in an OpenSim class.
  update-file  Update an .xml file (.osim or setup) to this version's format.
  viz          Show a model, motion, or data with the Simbody Visualizer.

  Pass -h or --help to any of these commands to learn how to use them.

Description of options:
  L, library  Load a plugin before executing the requested
              command. The <path> to the library can be absolute, or
              relative to the current directory. Make sure to include the
              library's extension (e.g., .dll, .so, .dylib). If <path>
              contains spaces, surround <path> in quotes. You can load
              multiple plugins by repeating this option.
  o, log      Control the verbosity of OpenSim's console output.
              Levels: off, critical, error, warn, info, debug, trace.
              Default: info.

Examples:
  opensim-cmd run-tool InverseDynamics_Setup.xml
  opensim-cmd print-xml cmc
  opensim-cmd info PathActuator
  opensim-cmd update-file lowerlimb_v3.3.osim lowerlimb_updated.osim
  opensim-cmd -L C:\Plugins\osimMyCustomForce.dll run-tool CMC_setup.xml
  opensim-cmd --library ../plugins/libosimMyPlugin.so print-xml MyCustomTool
  opensim-cmd --library=libosimMyCustomForce.dylib --log=debug info MyCustomForce

)";

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
    commands["viz"] = viz;

    // If no arguments are provided; just print the help text.
    // -------------------------------------------------------
    if (argc == 1) {
        std::cout << HELP << std::endl;
        return EXIT_SUCCESS;
    }

    // Parse the arguments.
    // --------------------
    std::map<std::string, docopt::value> args = OpenSim::parse_arguments(
            HELP, { argv + 1, argv + argc },
            true, // show help if requested
            "OpenSim " + GetVersionAndDate(),
            true); // take options first; necessary for nested commands.

    // Load the specified libraries.
    // -----------------------------
    if (args["--library"]) {
        for (const auto& plugin : args["--library"].asStringList()) {
            const bool success = LoadOpenSimLibraryExact(plugin);
            // Exit if we couldn't load the library.
            if (!success) return EXIT_FAILURE;
        }
    }

    // Logging.
    // --------
    if (args["--log"]) {
        Logger::setLevelString(args["--log"].asString());
    }

    // Did the user provide a valid command?
    // -------------------------------------
    if (!args["<command>"]) {
        log_error("No command provided.");
        return EXIT_FAILURE;
    }

    // The user provided a command.
    const auto& command = args["<command>"].asString();

    if (commands.count(command) == 0) {
        log_error("'{}' is not an opensim-cmd command. "
                  "See 'opensim-cmd --help'.", command);
        return EXIT_FAILURE;
    }

    // Dispatch to the requested command.
    // ----------------------------------
    // Each command returns whether or not it succeeded.
    return commands.at(command)(argc, argv);


    } catch (const std::exception& e) {
        log_error(e.what());
        return EXIT_FAILURE;
    }

    // We shouldn't ever get here, so we can consider this a failure.
    return EXIT_FAILURE;
}
