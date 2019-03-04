/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxWalkingStatelessMuscles.cpp                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

/// In this file, we attempt to solve
/// This file contains a problem we hope to solve robustly in the future.
/// It serves as a goal.
#include <Moco/osimMoco.h>

#include <OpenSim/OpenSim.h>

#include <OpenSim/Common/LogManager.h>

namespace OpenSim {

class MocoINDYGO : Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoINDYGO, Object);

public:
    OpenSim_DECLARE_PROPERTY(create_reserve_actuators, double,
            "Create a reserve actuator (CoordinateActuator) for each "
            "unconstrained "
            "coordinate in the model, and add each to the model. Each actuator "
            "will have the specified `optimal_force`, which should be set low "
            "to "
            "discourage the use of the reserve actuators. (default is -1, "
            "which "
            "means no reserves are created)");
    OpenSim_DECLARE_PROPERTY(external_loads_file, std::string,
            "TODO");
    MocoINDYGO() {
        constructProperty_create_reserve_actuators(-1);
        constructProperty_external_loads_file("");
    }

    void setModel(Model model) { m_model = std::move(model); }

    void setKinematicsFile(std::string fileName) {
        m_kinematicsFileName = std::move(fileName);
    }

    void setExternalLoadsFile(std::string fileName) {
        set_external_loads_file(std::move(fileName));
    }

    void solve() const {

        // TODO: Create an initial guess using GSO.

        Model model(m_model);

        MocoTool moco;
        auto& problem = moco.updProblem();

        model.finalizeFromProperties();
        for (const auto& muscle : model.getComponentList<Muscle>()) {
            if (!muscle.get_ignore_activation_dynamics()) {
                problem.setStateInfo(
                        muscle.getAbsolutePathString() + "/activation",
                        // TODO: Use the muscle's minimum_activation.
                        {0.01, 1});
                // TODO shouldn't be necessary.
                // problem.setStateInfo(
                //         muscle.getAbsolutePathString() + "/norm_fiber_length",
                //         {0.2, 1.8});
            }
        }

        InverseDynamicsTool idTool;
        if (!get_external_loads_file().empty()) {
            idTool.createExternalLoads(get_external_loads_file(), model);
        }

        model.initSystem();

        auto kinematicsRaw = STOFileAdapter::read(m_kinematicsFileName);
        auto kinematics = filterLowpass(kinematicsRaw, 6, true);

        auto posmot = PositionMotion::createFromTable(model, kinematics, true);
        posmot->setName("position_motion");
        model.addComponent(posmot.release());

        model.initSystem();
        if (get_create_reserve_actuators() != -1) {
            createReserveActuators(model, get_create_reserve_actuators());
        }

        problem.setModelCopy(model);

        // const double spaceForFiniteDiff = 1e-3;
        // problem.setTimeBounds(kinematicsRaw.getIndependentColumn().front() +
        //                               spaceForFiniteDiff,
        //         kinematicsRaw.getIndependentColumn().back() -
        //                 spaceForFiniteDiff);
        problem.setTimeBounds(0.01, 1.4);

        auto* effort = problem.addCost<MocoControlCost>("effort");
        effort->setWeight("reserve__jointset_ground_pelvis_pelvis_tilt", 0);
        effort->setWeight("reserve__jointset_ground_pelvis_pelvis_tx", 0);
        effort->setWeight("reserve__jointset_ground_pelvis_pelvis_ty", 0);

        // auto* stateTracking =
        //         problem.addCost<MocoStateTrackingCost>("tracking");
        // stateTracking->setReferenceFile(m_kinematicsFileName);

        auto& solver = moco.initCasADiSolver();
        solver.set_num_mesh_points(5);
        solver.set_dynamics_mode("implicit");
        solver.set_optim_convergence_tolerance(1e-3);
        solver.set_optim_constraint_tolerance(1e-2);
        solver.set_parallel(0);
        // solver.set_optim_ipopt_print_level(0);
        // solver.set_optim_hessian_approximation("exact");
        // TODO: Doesn't work yet.
        // solver.set_optim_sparsity_detection("random");
        // solver.set_optim_finite_difference_scheme("forward");

        // Storage kinSto(m_kinematicsFileName);
        // if (kinSto.isInDegrees()) {
        //     model.getSimbodyEngine().convertDegreesToRadians(kinSto);
        // }
        // auto guess = solver.createGuess();
        // guess.setStatesTrajectory(
        //         StatesTrajectory::createFromStatesStorage(model, kinSto, true)
        //                 .exportToTable(model));

        solver.setGuess(moco.solve());
        solver.set_num_mesh_points(50);
        MocoSolution solution = moco.solve().unseal();
        solution.write("sandboxWalkingStatelessMuscles_solution.sto");
        // moco.visualize(solution);
    }

private:
    Model m_model;
    std::string m_kinematicsFileName;

    static void createReserveActuators(Model& model, double optimalForce) {
        OPENSIM_THROW_IF(optimalForce <= 0, Exception,
                format("Invalid value (%g) for create_reserve_actuators; "
                       "should be -1 or positive.",
                        optimalForce));

        std::cout << "Adding reserve actuators with an optimal force of "
                  << optimalForce << "..." << std::endl;

        std::vector<std::string> coordPaths;
        // Borrowed from
        // CoordinateActuator::CreateForceSetOfCoordinateAct...
        for (const auto& coord : model.getComponentList<Coordinate>()) {
            auto* actu = new CoordinateActuator();
            actu->setCoordinate(&const_cast<Coordinate&>(coord));
            auto path = coord.getAbsolutePathString();
            coordPaths.push_back(path);
            // Get rid of model name.
            // Get rid of slashes in the path; slashes not allowed in names.
            std::replace(path.begin(), path.end(), '/', '_');
            actu->setName("reserve_" + path);
            actu->setOptimalForce(optimalForce);
            model.addComponent(actu);
        }
        // Re-make the system, since there are new actuators.
        model.initSystem();
        std::cout << "Added " << coordPaths.size()
                  << " reserve actuator(s), "
                     "for each of the following coordinates:"
                  << std::endl;
        for (const auto& name : coordPaths) {
            std::cout << "  " << name << std::endl;
        }
    }
};

} // namespace OpenSim

using namespace OpenSim;

int main() {
    try {
        std::cout.rdbuf(LogManager::cout.rdbuf());
        std::cerr.rdbuf(LogManager::cerr.rdbuf());
        Model model("testGait10dof18musc_subject01.osim");
        model.finalizeConnections();
        for (int im = 0; im < model.getMuscles().getSize(); ++im) {
            auto& muscle = model.getMuscles().get(im);
            // muscle.set_ignore_activation_dynamics(true);
            muscle.set_ignore_tendon_compliance(true);
        }
        DeGrooteFregly2016Muscle::replaceMuscles(model);

        MocoINDYGO indygo;
        indygo.setModel(model);
        indygo.setKinematicsFile("walk_gait1018_state_reference.mot");
        indygo.set_create_reserve_actuators(2.0);

        // indygo.setExternalLoadsFile("walk_gait1018_subject01_grf.xml");

        indygo.solve();
    } catch (const std::exception& e) { std::cerr << e.what() << std::endl; }

    // TODO: Implement cost minimization directly in CasADi.
    //      -> evaluating the integral cost only takes up like 5% of the
    //         computational time; the only benefit would be accurate
    //         derivatives.
    // TODO: External loads.
    // TODO: Activation dynamics.: can solve but takes way longer.
    //      Without, solves in 4 seconds.
    // TODO parallelization is changing the number of iterations for a
    // solution.
    // TODO: are results reproducible?? stochasticity in Millard model?
    // TODO: problem scaling.
    // TODO: solve with toy problem (single muscle).

    return EXIT_SUCCESS;
}
