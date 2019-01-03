#include <iostream>
#include <Moco/InverseMuscleSolver/GlobalStaticOptimization.h>
#include <Moco/InverseMuscleSolver/INDYGO.h>
#include <Moco/MocoTool.h>
#include <Moco/MocoProblem.h>
#include <Moco/MocoUtilities.h>

#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/osimSimulation.h>

using namespace OpenSim;

static const char helpMessage[] =
R"(OpenSim Moco. Use this command to run a setup file for the following:
  - Global Static Optimization,
  - INDYGO: Inverse, Dynamic, Global Optimization (tracking),
  - MocoTool: flexible optimal control framework (.omoco file).

Usage:
  opensim-moco -h | --help

    Print this help message.

  opensim-moco run-tool [--visualize] <setup-file>

    Run the tool specified in the provided setup file.
    Only MocoTool supports visualizing.

  opensim-moco print-xml <tool>

    Print a template XML file for the provided tool.
    <tool> can be "GlobalStaticOptimization", "INDYGO", or "MocoTool"

  opensim-moco visualize <model-or-omoco-file> [<iterate-file>]

    Visualize an OpenSim model (.osim file) with a MocoIterate, if provided.
    If an iterate is not provided, the model is visualized with its default
    state.
    You can provide a MocoTool setup file (.omoco) instead of a model.

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
                    className != "INDYGO" && className != "MocoTool") {
                throw Exception("Unexpected argument: " + className);
            }
            const auto* obj = Object::getDefaultInstanceOfType(argv[2]);
            if (!obj) {
                throw Exception("Cannot create an instance of " + className +
                        ".");
            }
            std::string fileName = "default_" + className;
            if (className == "MocoTool") fileName += ".omoco";
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
                    format("Unrecognized option '%s'; did you mean "
                           "'--visualize'?", arg2));
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
                    format("A problem occurred when trying to load file '%s'.",
                            setupFile));

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
            } else if (const auto* moco
                    = dynamic_cast<const MocoTool*>(obj.get())) {
                auto solution = moco->solve();
                if (visualize) moco->visualize(solution);
            } else {
                throw Exception("The provided file '" + setupFile +
                        "' yields a '" + obj->getConcreteClassName() +
                        "' but only GlobalStaticOptimization, INDYGO, and "
                                "MocoTool are acceptable.");
            }
        } else if (arg1 == "visualize") {
            OPENSIM_THROW_IF(argc < 3 || argc > 4, Exception,
                    "Incorrect number of arguments.");

            std::string file(argv[2]);
            std::unique_ptr<Model> model;
            if (file.rfind(".osim") != std::string::npos) {
                model = OpenSim::make_unique<Model>(file);
            } else {
                MocoTool moco(file);
                const MocoPhase& phase = moco.getProblem().getPhase(0);
                model.reset(phase.getModel().clone());
            }
            if (argc == 3) {
                // No motion provided.
                model->setUseVisualizer(true);
                auto state = model->initSystem();
                model->getVisualizer().show(state);
                std::cout << "Press any key to exit." << std::endl;
                // Wait for user input.
                std::cin.get();
            } else {
                MocoIterate iterate(argv[3]);
                visualize(*model, iterate.exportToStatesStorage());
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
