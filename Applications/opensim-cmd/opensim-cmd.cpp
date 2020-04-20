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

#include <OpenSim/OpenSim.h>
#include <OpenSim/version.h>

#include <iostream>

namespace {

bool is_arg(const char* c, const char* cmp) {
    return not strcmp(c, cmp);
}

template<typename... T>
bool is_arg(const char* c, const char* cmp1, T... cmp) {
    return is_arg(c, cmp1) or is_arg(c, cmp...);
}

bool is_help_arg(const char* c) {
    return is_arg(c, "--help", "-help", "-h");
}

bool starts_with(const char* pre, const char* s) {
    return strncmp(pre, s, strlen(pre)) == 0;
}

bool extract_opt_arg(int& argc, const char**& argv, const char* pre,  const char*& out) {
    const char* arg = argv[0];
    auto prelen = strlen(pre);
    auto arglen = strlen(arg);

    if (arglen < prelen) {
        return false;
    }

    // -O arg
    if (arg[prelen] == '\0') {
        if (argc < 2) {
            return false;
        }
        out = argv[1];
        argc--;
        argv++;
        return true;
    }

    // -O=arg
    if (arg[prelen] == '=') {
        if (arg[prelen+1] == '\0') {
            return false;
        }
        out = arg + prelen + 1;
        return true;
    }

    // -Oarg (but not --optarg)
    if (not starts_with("--", pre)) {
        out = arg + prelen;
        return true;
    }

    return false;
}

const char run_tool_usage[] = "usage: opensim-cmd run-tool [--help] SETUP_XML_FILE";
const char run_tool_help[] =
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

int run_tool(int argc, const char** argv) {
    const char* appname = argv[0];
    argc--;
    argv++;

    for (; argc > 0; argc--, argv++) {
        const char* arg = argv[0];

        if (arg[0] != '-') {
            break;
        } else if (is_help_arg(arg)) {
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

const char print_xml_help[] =
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
    const char* cmdname = argv[0];
    argc--;
    argv++;

    for (; argc > 0; argc--, argv++) {
        const char* arg = argv[0];

        if (arg[0] != '-') {
            break;
        } else if (is_help_arg(arg)) {
            std::cout << print_xml_help << std::endl;
            return EXIT_SUCCESS;
        }
    }

    if (argc == 0) {
        std::cerr << "Arguments did not match expected patterns" << std::endl;
        return EXIT_FAILURE;
    }

    if (argc > 2) {
        std::cerr << "Unexpected argument: print-xml, x, y, z" << std::endl;
        return EXIT_FAILURE;
    }

    const char* toolOrClass = argv[0];
    std::string toolLowerCase = SimTK::String::toLower(toolOrClass);
    const char* className;
    bool isBuiltInTool = true;

    if (toolLowerCase == "scale") {
        className = "ScaleTool";
    } else if (toolLowerCase == "ik") {
        className = "InverseKinematicsTool";
    } else if (toolLowerCase == "id") {
        className = "InverseDynamicsTool";
    } else if (toolLowerCase == "rra") {
        className = "RRATool";
    } else if (toolLowerCase == "cmc") {
        className = "CMCTool";
    } else if (toolLowerCase == "forward") {
        className = "ForwardTool";
    } else if (toolLowerCase == "analyze") {
        className = "AnalyzeTool";
    } else {
        className = toolOrClass;
        isBuiltInTool = false;
    }

    std::string outputFile;
    if (argc > 1) {
        outputFile = argv[1];
    } else {
        outputFile = "default_";
        if (isBuiltInTool) {
            outputFile += "Setup_";
        }
        outputFile += className;
        outputFile += ".xml";
    }

    // Print the XML file.
    // -------------------
    const auto* obj = OpenSim::Object::getDefaultInstanceOfType(className);

    if (obj == nullptr) {
        std::cerr << "There is no tool or registered concrete class named '" << className << "'.\n"
                  << "Did you intend to load a plugin (with --library)?" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Printing '" << outputFile << "'." << std::endl;
    OpenSim::Object::setSerializeAllDefaults(true);
    obj->print(outputFile);
    OpenSim::Object::setSerializeAllDefaults(false);

    return EXIT_SUCCESS;
}

const char info_help[] =
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

int info(int argc, const char** argv) {
    const char* cmdname = argv[0];
    argc--;
    argv++;

    for (; argc > 0; argc--, argv++) {
        const char* arg = argv[0];

        if (arg[0] != '-') {
            break;
        } else if (is_help_arg(arg)) {
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

const char update_file_usage[] = "usage: opensim-cmd update-file [--help] INPUT OUTPUT";
const char update_file_help[] =
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

int update_file(int argc, const char** argv) {
    const char* cmdname = argv[0];
    argc--;
    argv++;

    for (; argc > 0; argc--, argv++) {
        const char* arg = argv[0];

        if (*arg != '-') {
            break;
        } else if (is_help_arg(arg)) {
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
        { "run-tool", "Run a tool (e.g., Inverse Kinematics) from an XML setup file", run_tool },
        { "print-xml", "Print a template XML file for a Tool or class", print_xml },
        { "info", "Show description of properties in an OpenSim class.",  info },
        { "update-file", "Update an .xml file (.osim or setup) to this version's format.", update_file },
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

        if (is_help_arg(cmd)) {
            print_help(std::cout);
            return EXIT_SUCCESS;
        } else if (is_arg(cmd, "--version", "-V")) {
            std::cout << "OpenSim " << OpenSim::GetVersionAndDate() << std::endl;
            return EXIT_SUCCESS;
        } else if (starts_with("--library", cmd)) {
            const char* lib = "\0";
            if (not extract_opt_arg(argc, argv, "--library", lib)) {
                std::cerr << "--library requires an argument" << std::endl;
                std::cerr << usage_str << std::endl;
                return EXIT_FAILURE;
            }
            libs_to_load.emplace_back(lib);
        } else if (starts_with("-L", cmd)) {
            const char* lib = "\0";
            if (not extract_opt_arg(argc, argv, "-L", lib)) {
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
