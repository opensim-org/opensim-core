#ifndef OPENSIM_CMD_RUN_TOOL_H_
#define OPENSIM_CMD_RUN_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  opensim-cmd_run-tool.h                     *
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

static const char HELP_RUN_TOOL[] = 
R"(Run a tool (e.g., Inverse Kinematics) from an XML setup file.

Usage:
  opensim-cmd [options]... run-tool <setup-xml-file>
  opensim-cmd run-tool -h | --help

Options:
  -L <path>, --library <path>  Load a plugin.
  -o <level>, --log <level>  Logging level.

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
            MocoStudy

  This command will also recognize tools from plugins.

  Use `opensim-cmd print-xml` to generate a template <setup-xml-file>.

Examples:
  opensim-cmd run-tool CMC_setup.xml
  opensim-cmd -L C:\Plugins\osimMyCustomForce.dll run-tool CMC_setup.xml
  opensim-cmd --library ../plugins/libosimMyPlugin.so run-tool Forward_setup.xml
  opensim-cmd --library=libosimMyCustomForce.dylib run-tool CMC_setup.xml
)";

int run_tool(int argc, const char** argv) {

    using namespace OpenSim;

    std::map<std::string, docopt::value> args = OpenSim::parse_arguments(
            HELP_RUN_TOOL, { argv + 1, argv + argc },
            true); // show help if requested

    // Deserialize.
    const auto& setupFile = args["<setup-xml-file>"].asString();
    auto obj = std::unique_ptr<Object>(Object::makeObjectFromFile(setupFile));
    if (obj == nullptr) {
        throw Exception( "A problem occurred when trying to load file '" +
                setupFile + "'.");
    }

    // Detect and run the tool.
    if (auto* tool = dynamic_cast<AbstractTool*>(obj.get())) {
        // AbstractTool.
        // We must use the concrete class constructor, as it loads the model
        // (this also preserves the behavior of the previous command line
        // tools).
        log_info("Preparing to run {}.", tool->getConcreteClassName());
        std::unique_ptr<AbstractTool> concreteTool;
        if        (dynamic_cast<RRATool*>(tool)) {
            concreteTool.reset(new RRATool(setupFile));
        } else if (dynamic_cast<CMCTool*>(tool)) {
            concreteTool.reset(new CMCTool(setupFile));
        } else if (dynamic_cast<ForwardTool*>(tool)) {
            concreteTool.reset(new ForwardTool(setupFile));
        } else if (dynamic_cast<AnalyzeTool*>(tool)) {
            concreteTool.reset(new AnalyzeTool(setupFile));
        } else {
            log_warn("Detected an AbstractTool that is not RRA, "
                     "CMC, Forward, or Analyze; custom tools may not get "
                     "constructed properly.");
            concreteTool.reset(tool->clone());
        }
        const bool success = concreteTool->run();
        if (success) return EXIT_SUCCESS;
        else return EXIT_FAILURE;
    } else if (auto* tool = dynamic_cast<Tool*>(obj.get())) {
        // Tool.
        log_info("Preparing to run {}.", tool->getConcreteClassName());
        const bool success = tool->run();
        if (success) return EXIT_SUCCESS;
        else return EXIT_FAILURE;
    } else if (auto* scale = dynamic_cast<ScaleTool*>(obj.get())) {
        // ScaleTool.
        log_info("Preparing to run {}.", scale->getConcreteClassName());
        const bool success = scale->run();
        if (success) return EXIT_SUCCESS;
        else return EXIT_FAILURE;
    } else if (auto* study = dynamic_cast<MocoStudy*>(obj.get())) {
        log_info("Preparing to run {}.", study->getConcreteClassName());
        const auto solution = study->solve();
        if (solution.success()) return EXIT_SUCCESS;
        else return EXIT_FAILURE;

    } else {
        throw Exception("The provided file '" + setupFile + "' does not "
                "define an OpenSim Tool. Did you intend to load a plugin?");
    }
    return EXIT_FAILURE;
}

#endif // OPENSIM_CMD_RUN_TOOL_H_
