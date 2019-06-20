#include <Moco/InverseMuscleSolver/GlobalStaticOptimization.h>
#include <Moco/InverseMuscleSolver/INDYGO.h>
#include <Moco/MocoStudy.h>
#include <Moco/MocoProblem.h>
#include <Moco/MocoStudy.h>
#include <Moco/MocoUtilities.h>
#include <iostream>

#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/osimSimulation.h>

using namespace OpenSim;

static const char helpMessage[] =
        R"(OpenSim Moco. Use this command to run a setup file for the following:
  - Global Static Optimization,
  - INDYGO: Inverse, Dynamic, Global Optimization (tracking),
  - MocoStudy: flexible optimal control framework (.omoco file).

Usage:
  opensim-moco -h | --help

    Print this help message.

  opensim-moco [--library=<path>] run-tool [--visualize] <setup-file>

    Run the tool specified in the provided setup file.
    Only MocoStudy supports visualizing.

  opensim-moco [--library=<path>] print-xml <tool>

    Print a template XML file for the provided tool.
    <tool> can be "GlobalStaticOptimization", "INDYGO", or "MocoStudy"

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
            format("A problem occurred when trying to load file '%s'.",
                    setupFile));

    if (const auto* gso =
                    dynamic_cast<const GlobalStaticOptimization*>(obj.get())) {
        auto solution = gso->solve();
        if (visualize) std::cout << "Ignoring --visualize flag." << std::endl;
    } else if (const auto* mrs = dynamic_cast<const INDYGO*>(obj.get())) {
        auto solution = mrs->solve();
        if (visualize) std::cout << "Ignoring --visualize flag." << std::endl;
    } else if (const auto* moco = dynamic_cast<const MocoStudy*>(obj.get())) {
        auto solution = moco->solve();
        if (visualize) moco->visualize(solution);
    } else {
        throw Exception("The provided file '" + setupFile + "' yields a '" +
                        obj->getConcreteClassName() +
                        "' but only GlobalStaticOptimization, INDYGO, and "
                        "MocoStudy are acceptable.");
    }
}

void print_xml(std::string className) {
    if (className != "GlobalStaticOptimization" && className != "INDYGO" &&
            className != "MocoStudy") {
        throw Exception("Unexpected argument: " + className);
    }
    const auto* obj = Object::getDefaultInstanceOfType(className);
    if (!obj) {
        throw Exception("Cannot create an instance of " + className + ".");
    }
    std::string fileName = "default_" + className;
    if (className == "MocoStudy")
        fileName += ".omoco";
    else
        fileName += ".xml";
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
        MocoStudy moco(file);
        const MocoPhase& phase = moco.getProblem().getPhase(0);
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
        } else if (startsWith(arg1, "--library=")) {
            OpenSim::LoadOpenSimLibraryExact(arg1.substr(arg1.find("=") + 1));
            subcommand = argv[2];
            // Pretend we didn't get a library argument.
            --argc;
            offset = 1;
        } else {
            subcommand = arg1;
        }

        if (subcommand == "run-tool") {
            OPENSIM_THROW_IF(argc < 3 || argc > 4, Exception,
                    "Incorrect number of arguments.");

            std::string arg2(argv[2 + offset]);
            OPENSIM_THROW_IF(argc == 4 && arg2 != "--visualize", Exception,
                    format("Unrecognized option '%s'; did you mean "
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
                    argc != 3, Exception, "Incorrect number of arguments.");

            std::string className(argv[2 + offset]);
            print_xml(className);

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
