/* -------------------------------------------------------------------------- *
 * OpenSim Moco: opensim-moco.cpp                                             *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Moco/About.h>
#include <OpenSim/Moco/MocoProblem.h>
#include <OpenSim/Moco/MocoStudy.h>
#include <OpenSim/Moco/MocoUtilities.h>
#include <OpenSim/Simulation/osimSimulation.h>

#include <iostream>

using namespace OpenSim;

static const char helpMessage[] =
        R"(OpenSim Moco. Use this command to run a MocoStudy (.omoco file).

Usage:
  opensim-moco -h | --help
    Print this help message.

  opensim-moco -V | --version
    Print Moco's version.

  opensim-moco [--library=<path>] run [--visualize] <.omoco-file>
    Run the MocoStudy in the provided .omoco file.

  opensim-moco [--library=<path>] print-xml
    Print a template XML .omoco file for a MocoStudy.

  opensim-moco [--library=<path>] visualize <model-or-omoco-file> [<trajectory-file>]
    Visualize an OpenSim model (.osim file) with a MocoTrajectory, if provided.
    If a trajectory is not provided, the model is visualized with its default
    state.
    You can provide a MocoStudy setup file (.omoco) instead of a model.

  Use the --library flag to load a plugin.

)";

void run_tool(std::string setupFile, bool visualize) {

    auto obj = std::unique_ptr<Object>(Object::makeObjectFromFile(setupFile));

    OPENSIM_THROW_IF(obj == nullptr, Exception,
            "A problem occurred when trying to load file '{}'.", setupFile);

    if (const auto* moco = dynamic_cast<const MocoStudy*>(obj.get())) {
        auto solution = moco->solve();
        if (visualize) moco->visualize(solution);
    } else {
        throw Exception(
                fmt::format("The provided file '{}' yields a '{}' but a "
                            "MocoStudy was expected.",
                        setupFile, obj->getConcreteClassName()));
    }
}

void print_xml() {
    const auto* obj = Object::getDefaultInstanceOfType("MocoStudy");
    if (!obj) {
        throw Exception("Cannot create an instance of MocoStudy.");
    }
    std::string fileName = "default_MocoStudy.omoco";
    std::cout << "Printing '" << fileName << "'." << std::endl;
    Object::setSerializeAllDefaults(true);
    obj->print(fileName);
    Object::setSerializeAllDefaults(false);
}

void visualize(std::string file, std::string trajectory_file) {
    std::unique_ptr<Model> model;
    if (file.rfind(".osim") != std::string::npos) {
        model = OpenSim::make_unique<Model>(file);
    } else {
        MocoStudy study(file);
        const MocoPhase& phase = study.getProblem().getPhase(0);
        model.reset(phase.getModel().clone());
    }
    if (trajectory_file.empty()) {
        // No motion provided.
        model->setUseVisualizer(true);
        auto state = model->initSystem();
        model->getVisualizer().show(state);
        std::cout << "Press any key to exit." << std::endl;
        // Wait for user input.
        std::cin.get();
    } else {
        MocoTrajectory trajectory(trajectory_file);
        visualize(*model, trajectory.exportToStatesStorage());
    }
}


int main(int argc, char* argv[]) {

    try {

        if (argc == 1) {
            std::cout << helpMessage << std::endl;
            return EXIT_SUCCESS;
        }

        std::string arg1(argv[1]);
        std::string subcommand;
        int offset = 0;
        if (arg1 == "-h" || arg1 == "--help") {
            std::cout << helpMessage << std::endl;
            return EXIT_SUCCESS;
        } else if (arg1 == "-V" || arg1 == "--version") {
            std::cout << OpenSim::GetMocoVersion() << std::endl;
            return EXIT_SUCCESS;
        } else if (startsWith(arg1, "--library=")) {
            OpenSim::LoadOpenSimLibraryExact(arg1.substr(arg1.find("=") + 1));
            subcommand = argv[2];
            // Pretend we didn't get a library argument.
            --argc;
            offset = 1;
        } else {
            subcommand = arg1;
        }

        if (subcommand == "run") {
            OPENSIM_THROW_IF(argc < 3 || argc > 4, Exception,
                    "Incorrect number of arguments.");

            std::string arg2(argv[2 + offset]);
            OPENSIM_THROW_IF(argc == 4 && arg2 != "--visualize", Exception,
                    fmt::format("Unrecognized option '{}'; did you mean "
                                "'--visualize'?",
                            arg2));
            std::string setupFile;
            bool visualize = false;
            if (argc == 3) {
                setupFile = arg2;
            } else {
                setupFile = std::string(argv[3 + offset]);
                visualize = true;
            }
            run_tool(setupFile, visualize);

        } else if (subcommand == "print-xml") {
            OPENSIM_THROW_IF(
                    argc != 2, Exception, "Incorrect number of arguments.");

            print_xml();

        } else if (subcommand == "visualize") {
            OPENSIM_THROW_IF(argc < 3 || argc > 4, Exception,
                    "Incorrect number of arguments.");

            std::string file(argv[2 + offset]);
            std::string trajectory;
            if (argc == 4) {
                trajectory = argv[3 + offset];
            }
            visualize(file, trajectory);

        } else {
            std::cout << "Unrecognized arguments. See usage with -h or --help"
                         "."
                      << std::endl;
        }
    } catch (const std::exception& exc) {
        std::cout << exc.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
