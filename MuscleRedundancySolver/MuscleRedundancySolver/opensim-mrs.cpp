#include <iostream>
#include "GlobalStaticOptimizationSolver.h"
#include "MuscleRedundancySolver.h"

using namespace OpenSim;

// TODO
//const std::string helpMessage =
//R("OpenSim Inverse Muscle Solvers (Global Static Optimization and Muscle "
//        "Redundancy Solver");
int main(int argc, char* argv[]) {

    try {
        OPENSIM_THROW_IF(argc != 2, Exception,
            "opensim-mrs requires exactly 1 argument (path to setup file).");

        Object::registerType(GlobalStaticOptimizationSolver());
        Object::registerType(MuscleRedundancySolver());

        const std::string setupFile(argv[1]);
        auto obj = std::unique_ptr<Object>(Object::makeObjectFromFile(setupFile));

        OPENSIM_THROW_IF(obj == nullptr, Exception,
            "A problem occurred when trying to load file '" + setupFile + "'.");

        if (const auto* gso =
                dynamic_cast<const GlobalStaticOptimizationSolver*>(obj.get()))
        {
            auto solution = gso->solve();
        } else if (const auto* mrs =
                dynamic_cast<const MuscleRedundancySolver*>(obj.get())) {
            auto solution = mrs->solve();
        } else {
            throw Exception("The provided file '" + setupFile + "' does not "
                    "define an InverseMuscleSolver.");
        }

    } catch (const std::exception& exc) {
        std::cout << exc.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
