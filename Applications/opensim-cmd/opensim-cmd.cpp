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

#include "subcommands/info.h"
#include "subcommands/print-xml.h"
#include "subcommands/run-tool.h"
#include "subcommands/update-file.h"
#include "cli_helpers.h"

#include <OpenSim/OpenSim.h>
#include <OpenSim/version.h>

#include <iostream>

namespace {
struct SubCommand {
    const char* name;
    const char* description;
    int(*command_func)(int, const char**);
};

const char usage_str[] = "usage: opensim-cmd [--version] [--help] COMMAND [<args>]";
const char help_head[] =
        R"(OpenSim: musculoskeletal modeling and simulation.
Usage:
  opensim-cmd [--library=<path>]... <command> [<args>...]
  opensim-cmd -h | --help
  opensim-cmd -V | --version

Options:
  -L <path>, --library <path>  Load a plugin before executing the requested
                 command. The <path> to the library can be absolute, or
                 relative to the current directory. Make sure to include the
                 library's extension (e.g., .dll, .so, .dylib). If <path>
                 contains spaces, surround <path> in quotes. You can load
                 multiple plugins by repeating this option.
  -h, --help     Show this help description.
  -V, --version  Show the version number.)";

const SubCommand subcommands[] = {
        { "run-tool", "Run a tool (e.g., Inverse Kinematics) from an XML setup file", OpenSimCmd::run_tool },
        { "print-xml", "Print a template XML file for a Tool or class", OpenSimCmd::print_xml },
        { "info", "Show description of properties in an OpenSim class.",  OpenSimCmd::info },
        { "update-file", "Update an .xml file (.osim or setup) to this version's format.", OpenSimCmd::update_file },
};
const char help_foot[] =
        R"(Examples:
  opensim-cmd run-tool InverseDynamics_Setup.xml
  opensim-cmd print-xml cmc
  opensim-cmd info PathActuator\n
  opensim-cmd update-file lowerlimb_v3.3.osim lowerlimb_updated.osim
  opensim-cmd -L C:\\Plugins\\osimMyCustomForce.dll run-tool CMC_setup.xml
  opensim-cmd --library ../plugins/libosimMyPlugin.so print-xml MyCustomTool
  opensim-cmd --library=libosimMyCustomForce.dylib info MyCustomForce)";

void print_help(std::ostream& out) {
    out << help_head << std::endl;

    out << std::endl;
    out << "Available commands:" << std::endl;
    for (const SubCommand& cmd : subcommands) {
        out << "  " << cmd.name << cmd.description << std::endl;
    }
    out << std::endl;
    out << "  Pass -h or --help to any of these commands to learn how to use them." << std::endl;

    out << help_foot << std::endl;
}
}

int main(int argc, const char** argv) {
    const char* appname = argv[0];
    argc--;
    argv++;

    std::vector<std::string> libs_to_load;
    for (; argc > 0; argc--, argv++) {
        const char* cmd = argv[0];

        if (cmd[0] != '-') {
            break;
        }

        if (OpenSimCmd::is_help_arg(cmd)) {
            print_help(std::cout);
            return EXIT_SUCCESS;
        } else if (OpenSimCmd::is_arg(cmd, "--version", "-V")) {
            std::cout << "OpenSim " << OpenSim::GetVersionAndDate() << std::endl;
            return EXIT_SUCCESS;
        } else if (OpenSimCmd::starts_with("--library", cmd)) {
            const char* lib = "\0";
            if (not OpenSimCmd::extract_opt_arg(argc, argv, "--library", lib)) {
                std::cerr << "--library requires an argument" << std::endl;
                std::cerr << usage_str << std::endl;
                return EXIT_FAILURE;
            }
            libs_to_load.emplace_back(lib);
        } else if (OpenSimCmd::starts_with("-L", cmd)) {
            const char* lib = "\0";
            if (not OpenSimCmd::extract_opt_arg(argc, argv, "-L", lib)) {
                std::cerr << "-L requires an argument" << std::endl;
                std::cerr << usage_str << std::endl;
                return EXIT_FAILURE;
            }
            libs_to_load.emplace_back(lib);
        }
    }

    if (argc == 0) {
        if (not libs_to_load.empty()) {
            std::cerr << "Arguments did not match expected patterns" << std::endl;
            print_help(std::cerr);
            return EXIT_FAILURE;
        } else {
            print_help(std::cerr);
            // TODO: this shouldn't be necessary, but is required to pass the test.
            return EXIT_SUCCESS;
        }
    }

    const char* subcmd = argv[0];

    for (const SubCommand& cmd : subcommands) {
        if (not strcmp(cmd.name, subcmd)) {
            for (const std::string& lib : libs_to_load) {
                if (not OpenSim::LoadOpenSimLibraryExact(lib)) {
                    return EXIT_FAILURE;
                }
            }
            try {
                return cmd.command_func(argc, argv);
            } catch (const std::exception& ex) {
                std::cerr << ex.what() << std::endl;
                return EXIT_FAILURE;
            }
        }
    }

    std::cerr << "'" << subcmd << "' is not an opensim-cmd command. See 'opensim-cmd --help'." << std::endl;
    return EXIT_FAILURE;
}
