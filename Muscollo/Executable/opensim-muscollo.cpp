#include <iostream>
#include <Muscollo/InverseMuscleSolver/GlobalStaticOptimization.h>
#include <Muscollo/InverseMuscleSolver/INDYGO.h>
#include <Muscollo/MucoTool.h>
#include <Muscollo/MucoProblem.h>
#include <Muscollo/MuscolloUtilities.h>

#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/osimSimulation.h>

using namespace OpenSim;

static const char helpMessage[] =
R"(OpenSim Muscollo. Use this command to run a setup file for the following:
  - Global Static Optimization,
  - INDYGO: Inverse, Dynamic, Global Optimization (tracking),
  - MucoTool: flexible optimal control framework (.omuco file).

Usage:
  opensim-muscollo -h | --help

    Print this help message.

  opensim-muscollo run-tool [--visualize] <setup-file>

    Run the tool specified in the provided setup file.
    Only MucoTool supports visualizing.

  opensim-muscollo print-xml <tool>

    Print a template XML file for the provided tool.
    <tool> can be "GlobalStaticOptimization", "INDYGO", or "MucoTool"

  opensim-muscollo visualize <model-file> [<iterate-file>]

    Visualize an OpenSim model (.osim file) with a MucoIterate, if provided.

)";

int main(int argc, char* argv[]) {

    try {

        if (argc == 1) {
            std::cout << helpMessage << std::endl;
            return EXIT_SUCCESS;
        }

        std::string arg1(argv[1]);
        if (arg1 == "-h" || arg1 == "--help") {
            std::cout << helpMessage << std::endl;
            return EXIT_SUCCESS;
        }

        if (arg1 == "print-xml") {
            OPENSIM_THROW_IF(argc != 3, Exception,
                    "Incorrect number of arguments.");

            std::string className(argv[2]);
            if (className != "GlobalStaticOptimization" &&
                    className != "INDYGO" && className != "MucoTool") {
                throw Exception("Unexpected argument: " + className);
            }
            const auto* obj = Object::getDefaultInstanceOfType(argv[2]);
            if (!obj) {
                throw Exception("Cannot create an instance of " + className +
                        ".");
            }
            std::string fileName = "default_" + className;
            if (className == "MucoTool") fileName += ".omuco";
            else fileName += ".xml";
            std::cout << "Printing '" << fileName << "'." << std::endl;
            Object::setSerializeAllDefaults(true);
            obj->print(fileName);
            Object::setSerializeAllDefaults(false);

        } else if (arg1 == "run-tool") {
            OPENSIM_THROW_IF(argc < 3 || argc > 4, Exception,
                    "Incorrect number of arguments.");

            std::string arg2(argv[2]);
            OPENSIM_THROW_IF(argc == 4 && arg2 != "--visualize", Exception,
                    "Unrecognized option '" + arg2 +
                    "'; did you mean '--visualize'?");
            std::string setupFile;
            bool visualize = false;
            if (argc == 3) {
                setupFile = arg2;
            } else {
                setupFile = std::string(argv[3]);
                visualize = true;
            }

            auto obj = std::unique_ptr<Object>(
                    Object::makeObjectFromFile(setupFile));

            OPENSIM_THROW_IF(obj == nullptr, Exception,
                    "A problem occurred when trying to load file '" + setupFile
                            + "'.");

            if (const auto* gso =
                    dynamic_cast<const GlobalStaticOptimization*>(obj.get())) {
                auto solution = gso->solve();
                if (visualize)
                    std::cout << "Ignoring --visualize flag." << std::endl;
            } else if (const auto* mrs =
                    dynamic_cast<const INDYGO*>(obj.get())) {
                auto solution = mrs->solve();
                if (visualize)
                    std::cout << "Ignoring --visualize flag." << std::endl;
            } else if (const auto* muco
                    = dynamic_cast<const MucoTool*>(obj.get())) {
                auto solution = muco->solve();
                if (visualize) muco->visualize(solution);
            } else {
                throw Exception("The provided file '" + setupFile +
                        "' yields a '" + obj->getConcreteClassName() +
                        "' but only GlobalStaticOptimization, INDYGO, and "
                                "MucoTool are acceptable.");
            }
        } else if (arg1 == "visualize") {
            OPENSIM_THROW_IF(argc < 3 || argc > 4, Exception,
                    "Incorrect number of arguments.");

            std::string modelFile(argv[2]);
            if (argc == 3) {
                // No motion provided.
                Model model(modelFile);
                model.setUseVisualizer(true);
                auto state = model.initSystem();
                model.getVisualizer().show(state);
                std::cout << "Press any key to exit." << std::endl;
                // Wait for user input.
                std::cin.get();
            } else {
                MucoIterate iterate(argv[3]);
                visualize(Model(modelFile), iterate.exportToStatesStorage());
            }
        } else {
            std::cout << "Unrecognized arguments. See usage with -h or --help"
                    "." << std::endl;
        }

    } catch (const std::exception& exc) {
        std::cout << exc.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
