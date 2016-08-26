#ifndef OPENSIM_RUN_TOOL_H_
#define OPENSIM_RUN_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  opensim_run_tool.h                         *
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

#include <iostream>

#include <docopt.h>
#include "parse_arguments.h"

#include <OpenSim/OpenSim.h>

static const char HELP_RUN_TOOL[] = 
R"(Run a tool (e.g., Inverse Kinematics) from an XML setup file.

Usage:
  opensim [options]... run-tool <setup-xml-file>
  opensim run-tool -h | --help

Options:
  -L <path>, --library <path>  Load a plugin.

Description:
  The Tool to run is detected from the setup file you provide. Supported tools
  include the following:
  
            Scale
            Inverse Kinematics           (IK)
            Inverse Dynamics             (ID)
            Residual Reduction Algorithm (RRA)
            Computed Muscle Control      (CMC)
            Forward                      
            Analyze

  This command will also recognize tools from plugins.

  Use `opensim print-xml` to generate a template <setup-xml-file>.

Examples:
  opensim run-tool CMC_setup.xml
  opensim -L C:\Plugins\osimMyCustomForce.dll run-tool CMC_setup.xml
  opensim --library ../plugins/libosimMyPlugin.so run-tool Forward_setup.xml
  opensim --library=libosimMyCustomForce.dylib run-tool CMC_setup.xml
)";

int run_tool(int argc, const char** argv) {

    using namespace OpenSim;

    std::map<std::string, docopt::value> args = OpenSim::parse_arguments(
            HELP_RUN_TOOL, { argv + 1, argv + argc },
            true); // show help if requested

    // Deserialize.
    const auto& setupFile = args["<setup-xml-file>"].asString();
    Object* obj = Object::makeObjectFromFile(setupFile);
    if (obj == nullptr) {
        throw Exception( "A problem occured when trying to load file '" +
                setupFile + "'.");
    }

    // Detect and run the tool.
    if (auto* tool = dynamic_cast<AbstractTool*>(obj)) {
        // AbstractTool.
        const bool success = tool->run();
        if (success) return EXIT_SUCCESS;
        else return EXIT_FAILURE;
    } else if (auto* tool = dynamic_cast<Tool*>(obj)) {
        // Tool.
        const bool success = tool->run();
        if (success) return EXIT_SUCCESS;
        else return EXIT_FAILURE;
    } else if (auto* scale = dynamic_cast<ScaleTool*>(obj)) {
        // ScaleTool.
        const bool success = scale->run();
        if (success) return EXIT_SUCCESS;
        else return EXIT_FAILURE;
    } else {
        throw Exception("The provided file '" + setupFile + "' does not "
                "define an OpenSim Tool. Did you intend to load a plugin?");
    }
    return EXIT_FAILURE;
}

#endif // OPENSIM_RUN_TOOL_H_
