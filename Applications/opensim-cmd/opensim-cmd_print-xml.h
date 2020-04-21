#ifndef OPENSIM_CMD_PRINT_XML_H_
#define OPENSIM_CMD_PRINT_XML_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  opensim-cmd_print-xml.h                    *
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

#include <OpenSim/OpenSim.h>

static const char HELP_PRINT_XML[] =
R"(Print a template XML file for a Tool or class.

Usage:
  opensim-cmd [options]... print-xml <tool-or-class> [<output-file>]
  opensim-cmd print-xml -h | --help

Options:
  -L <path>, --library <path>  Load a plugin.

Description:
  The argument <tool-or-class> can be the name of a Tool
  
         scale  ik  id  rra  cmc  forward  analyze     (case-insensitive)

  or the name of any registered (concrete) OpenSim class (even from a plugin).
  Here are descriptions of the Tools listed above:
  
         scale    Create a subject-specific model.
         ik       Inverse Kinematics
         id       Inverse Dynamics
         rra      Residual Reduction Algorithm
         cmc      Computed Muscle Control
         forward  Perform a forward simulation, using any controllers.
         analyze  Obtain muscle-related quantites, joint loads; 
                  perform Static Optimization; etc.

  The template file is written to <output-file> if provided. Otherwise, the
  file is written to the current directory with the name
  `default_Setup_<tool-class-name>.xml` when given a Tool name, or 
  `default_<class-name>.xml` otherwise.

  You can run a Tool setup file with `opensim-cmd run-tool`.

Examples:
  opensim-cmd print-xml cmc
  opensim-cmd print-xml Analyze
  opensim-cmd print-xml Millard2012EquilibriumMuscle 
)";

int print_xml(int argc, const char** argv) {

    using namespace OpenSim;

    std::map<std::string, docopt::value> args = OpenSim::parse_arguments(
            HELP_PRINT_XML, { argv + 1, argv + argc },
            true); // show help if requested

    // Parse arguments.
    // ----------------

    // Tool or class.
    std::string toolOrClass = args["<tool-or-class>"].asString();
    std::string toolLowerCase = SimTK::String::toLower(toolOrClass);
    std::string className;
    bool isBuiltInTool = true;
    if      (toolLowerCase == "scale")   className = "ScaleTool";
    else if (toolLowerCase == "ik")      className = "InverseKinematicsTool";
    else if (toolLowerCase == "id")      className = "InverseDynamicsTool";
    else if (toolLowerCase == "rra")     className = "RRATool";
    else if (toolLowerCase == "cmc")     className = "CMCTool";
    else if (toolLowerCase == "forward") className = "ForwardTool";
    else if (toolLowerCase == "analyze") className = "AnalyzeTool";
    else {
        className = toolOrClass;
        isBuiltInTool = false;
    }

    // Output file.
    std::string outputFile;
    if (args["<output-file>"]) {
        outputFile = args["<output-file>"].asString();
    } else {
        outputFile = "default_";
        if (isBuiltInTool) outputFile += "Setup_";
        outputFile += className + ".xml";
    }


    // Print the XML file.
    // -------------------
    const auto* obj = Object::getDefaultInstanceOfType(className);

    if (!obj) {
        throw Exception("There is no tool or registered concrete class named '"
                + className + "'.\n"
                "Did you intend to load a plugin (with --library)?");
    }

    std::cout << "Printing '" << outputFile << "'." << std::endl;
    Object::setSerializeAllDefaults(true);
    obj->print(outputFile);
    Object::setSerializeAllDefaults(false);

    return EXIT_SUCCESS;
}

#endif // OPENSIM_CMD_PRINT_XML_H_
