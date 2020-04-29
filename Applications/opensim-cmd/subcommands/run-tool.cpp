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

#include "run-tool.h"

#include "Applications/opensim-cmd/cli_helpers.h"

#include <OpenSim/OpenSim.h>

#include <iostream>

static const char run_tool_usage[] = "usage: opensim-cmd run-tool [--help] SETUP_XML_FILE";
static const char run_tool_help[] =
        R"(Run a tool (e.g., Inverse Kinematics) from an XML setup file.

options:
  --help                      Print this help message and exit

description:
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

  Use `opensim-cmd print-xml` to generate a template <setup-xml-file>.

examples:
  opensim-cmd run-tool CMC_setup.xml)";

int OpenSimCmd::run_tool(int argc, const char** argv) {
    const char* appname = argv[0];
    argc--;
    argv++;

    for (; argc > 0; argc--, argv++) {
        const char* arg = argv[0];

        if (arg[0] != '-') {
            break;
        } else if (OpenSimCmd::is_help_arg(arg)) {
            std::cout << run_tool_help << std::endl;
            return EXIT_SUCCESS;
        }
    }

    if (argc == 0) {
        std::cerr << "Arguments did not match expected patterns" << std::endl;
        std::cerr << run_tool_help << std::endl;
        return EXIT_FAILURE;
    }

    if (argc > 1) {
        std::cerr << appname << ": multiple SETUP_XML_FILE arguments provided (only one supported)" << std::endl;
        std::cerr << run_tool_usage << std::endl;
        return EXIT_FAILURE;
    }

    using namespace OpenSim;

    // Deserialize.
    std::string setupFile{argv[0]};
    std::unique_ptr<Object> obj{Object::makeObjectFromFile(setupFile)};

    if (obj == nullptr) {
        std::cerr << appname
                  << setupFile
                  << ": a problem occurred when trying to load file"
                  << std::endl;
        return EXIT_FAILURE;
    }

    // Detect and run the tool.
    if (auto* tool = dynamic_cast<AbstractTool*>(obj.get())) {
        // AbstractTool.
        // We must use the concrete class constructor, as it loads the model
        // (this also preserves the behavior of the previous command line
        // tools).
        std::cout << "Preparing to run " << tool->getConcreteClassName() << "." << std::endl;
        std::unique_ptr<AbstractTool> concreteTool;
        if (dynamic_cast<RRATool*>(tool)) {
            concreteTool.reset(new RRATool(setupFile));
        } else if (dynamic_cast<CMCTool*>(tool)) {
            concreteTool.reset(new CMCTool(setupFile));
        } else if (dynamic_cast<ForwardTool*>(tool)) {
            concreteTool.reset(new ForwardTool(setupFile));
        } else if (dynamic_cast<AnalyzeTool*>(tool)) {
            concreteTool.reset(new AnalyzeTool(setupFile));
        } else {
            std::cout << "Detected an AbstractTool that is not RRA, "
                         "CMC, Forward, or Analyze; custom tools may not get "
                         "constructed properly."
                      << std::endl;
            concreteTool.reset(tool->clone());
        }

        return concreteTool->run() ? EXIT_SUCCESS : EXIT_FAILURE;
    } else if (auto* tool = dynamic_cast<Tool*>(obj.get())) {
        // Tool.
        std::cout << "Preparing to run " << tool->getConcreteClassName() << "." << std::endl;
        return tool->run() ? EXIT_SUCCESS : EXIT_FAILURE;
    } else if (auto* scale = dynamic_cast<ScaleTool*>(obj.get())) {
        // ScaleTool.
        std::cout << "Preparing to run " << scale->getConcreteClassName() << "." << std::endl;
        return scale->run() ? EXIT_SUCCESS : EXIT_FAILURE;
    } else {
        std::cerr << "The provided file '"
                  << setupFile
                  << "' does not define an OpenSim Tool. Did you intend to load a plugin?"
                  << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_FAILURE;
}
