/* -------------------------------------------------------------------------- *
 * OpenSim Moco: example2DWalkingMetabolics.cpp                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-20 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
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

/// This example features a tracking simulation of walking that includes
/// minimization of the metabolic cost of transport computed using a smooth
/// approximation of the metabolic energy model of Bhargava et al (2004).
///
/// The code is inspired from Falisse A, Serrancoli G, Dembia C, Gillis J,
/// De Groote F: Algorithmic differentiation improves the computational
/// efficiency of OpenSim-based trajectory optimization of human movement.
/// PLOS One, 2019.
///
/// Model
/// -----
/// The model described in the file '2D_gait.osim' included in this file is a
/// modified version of the 'gait10dof18musc.osim' available within OpenSim. We
/// replaced the moving knee flexion axis by a fixed flexion axis, replaced the
/// Millard2012EquilibriumMuscles by DeGrooteFregly2016Muscles, and added
/// SmoothSphereHalfSpaceForces (two contacts per foot) to model the
/// contact interactions between the feet and the ground.
///
/// Data
/// ----
/// The coordinate data included in the 'referenceCoordinates.sto' comes from
/// predictive simulations generated in Falisse et al. 2019.

#include <OpenSim/Moco/MocoGoal/MocoOutputGoal.h>
#include <OpenSim/Moco/osimMoco.h>

using namespace OpenSim;

// Set a coordinate tracking problem where the goal is to minimize the
// difference between provided and simulated coordinate values and speeds
// as well as to minimize an effort cost (squared controls) and a metabolic
// cost (metabolic energy rate normalized by distance traveled and body mass;
// the metabolics model is based on a smooth approximation of the
// phenomenological model described by Bhargava et al. (2004)). The provided
// data represents half a gait cycle. Endpoint constraints enforce periodicity
// of the coordinate values (except for pelvis tx) and speeds, coordinate
// actuator controls, and muscle activations.
void gaitTrackingMetabolics() {

    using SimTK::Pi;

    MocoTrack track;
    track.setName("gaitTrackingMetabolics");

    // Define the optimal control problem.
    // ===================================
    Model baseModel("2D_gait.osim");

    // Add metabolics
    Bhargava2004SmoothedMuscleMetabolics* metabolics =
            new Bhargava2004SmoothedMuscleMetabolics();
    metabolics->setName("metabolics");
    metabolics->set_use_smoothing(true);
    metabolics->addMuscle("hamstrings_r",
            baseModel.getComponent<Muscle>("hamstrings_r"));
    metabolics->addMuscle("hamstrings_l",
            baseModel.getComponent<Muscle>("hamstrings_l"));
    metabolics->addMuscle("bifemsh_r",
            baseModel.getComponent<Muscle>("bifemsh_r"));
    metabolics->addMuscle("bifemsh_l",
            baseModel.getComponent<Muscle>("bifemsh_l"));
    metabolics->addMuscle("glut_max_r",
            baseModel.getComponent<Muscle>("glut_max_r"));
    metabolics->addMuscle("glut_max_l",
            baseModel.getComponent<Muscle>("glut_max_l"));
    metabolics->addMuscle("iliopsoas_r",
            baseModel.getComponent<Muscle>("iliopsoas_r"));
    metabolics->addMuscle("iliopsoas_l",
            baseModel.getComponent<Muscle>("iliopsoas_l"));
    metabolics->addMuscle("rect_fem_r",
            baseModel.getComponent<Muscle>("rect_fem_r"));
    metabolics->addMuscle("rect_fem_l",
            baseModel.getComponent<Muscle>("rect_fem_l"));
    metabolics->addMuscle("vasti_r",
            baseModel.getComponent<Muscle>("vasti_r"));
    metabolics->addMuscle("vasti_l",
            baseModel.getComponent<Muscle>("vasti_l"));
    metabolics->addMuscle("gastroc_r",
            baseModel.getComponent<Muscle>("gastroc_r"));
    metabolics->addMuscle("gastroc_l",
            baseModel.getComponent<Muscle>("gastroc_l"));
    metabolics->addMuscle("soleus_r",
            baseModel.getComponent<Muscle>("soleus_r"));
    metabolics->addMuscle("soleus_l",
            baseModel.getComponent<Muscle>("soleus_l"));
    metabolics->addMuscle("tib_ant_r",
            baseModel.getComponent<Muscle>("tib_ant_r"));
    metabolics->addMuscle("tib_ant_l",
            baseModel.getComponent<Muscle>("tib_ant_l"));
    baseModel.addComponent(metabolics);
    baseModel.finalizeConnections();

    ModelProcessor modelprocessor = ModelProcessor(baseModel);
    track.setModel(modelprocessor);
    track.setStatesReference(
            TableProcessor("referenceCoordinates.sto") | TabOpLowPassFilter(6));
    track.set_states_global_tracking_weight(30.0);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
    track.set_apply_tracked_states_to_guess(true);
    track.set_initial_time(0.0);
    track.set_final_time(0.47008941);
    MocoStudy study = track.initialize();
    MocoProblem& problem = study.updProblem();

    // Goals.
    // =====
    // Symmetry.
    auto* symmetryGoal = problem.addGoal<MocoPeriodicityGoal>("symmetryGoal");
    Model model = modelprocessor.process();
    model.initSystem();
    // Symmetric coordinate values (except for pelvis_tx) and speeds.
    for (const auto& coord : model.getComponentList<Coordinate>()) {
        if (IO::EndsWith(coord.getName(), "_r")) {
            symmetryGoal->addStatePair({coord.getStateVariableNames()[0],
                    std::regex_replace(coord.getStateVariableNames()[0],
                            std::regex("_r"), "_l")});
            symmetryGoal->addStatePair({coord.getStateVariableNames()[1],
                    std::regex_replace(coord.getStateVariableNames()[1],
                            std::regex("_r"), "_l")});
        }
        if (IO::EndsWith(coord.getName(), "_l")) {
            symmetryGoal->addStatePair({coord.getStateVariableNames()[0],
                    std::regex_replace(coord.getStateVariableNames()[0],
                            std::regex("_l"), "_r")});
            symmetryGoal->addStatePair({coord.getStateVariableNames()[1],
                    std::regex_replace(coord.getStateVariableNames()[1],
                            std::regex("_l"), "_r")});
        }
        if (!IO::EndsWith(coord.getName(), "_l") &&
                !IO::EndsWith(coord.getName(), "_r") &&
                !IO::EndsWith(coord.getName(), "_tx")) {
            symmetryGoal->addStatePair({coord.getStateVariableNames()[0],
                    coord.getStateVariableNames()[0]});
            symmetryGoal->addStatePair({coord.getStateVariableNames()[1],
                    coord.getStateVariableNames()[1]});
        }
    }
    symmetryGoal->addStatePair({"/jointset/groundPelvis/pelvis_tx/speed"});
    // Symmetric coordinate actuator controls.
    symmetryGoal->addControlPair({"/lumbarAct"});
    // Symmetric muscle activations.
    for (const auto& muscle : model.getComponentList<Muscle>()) {
        if (IO::EndsWith(muscle.getName(), "_r")) {
            symmetryGoal->addStatePair({muscle.getStateVariableNames()[0],
                    std::regex_replace(muscle.getStateVariableNames()[0],
                            std::regex("_r"), "_l")});
        }
        if (IO::EndsWith(muscle.getName(), "_l")) {
            symmetryGoal->addStatePair({muscle.getStateVariableNames()[0],
                    std::regex_replace(muscle.getStateVariableNames()[0],
                            std::regex("_l"), "_r")});
        }
    }
    // Effort. Get a reference to the MocoControlGoal that is added to every
    // MocoTrack problem by default.
    MocoControlGoal& effort =
            dynamic_cast<MocoControlGoal&>(problem.updGoal("control_effort"));
    effort.setWeight(0.1);
    // Metabolics; total metabolic rate includes activation heat rate,
    // maintenance heat rate, shortening heat rate, mechanical work rate, and
    // basal metabolic rate.
    auto* metGoal = problem.addGoal<MocoOutputGoal>("met", 0.1);
    metGoal->setOutputPath("/metabolics|total_metabolic_rate");
    metGoal->setDivideByDisplacement(true);
    metGoal->setDivideByMass(true);

    // Bounds.
    // =======
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value",
            {-20 * Pi / 180, -10 * Pi / 180});
    problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value", {0, 1});
    problem.setStateInfo(
            "/jointset/groundPelvis/pelvis_ty/value", {0.75, 1.25});
    problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",
            {-10 * Pi / 180, 60 * Pi / 180});
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
            {-10 * Pi / 180, 60 * Pi / 180});
    problem.setStateInfo(
            "/jointset/knee_l/knee_angle_l/value", {-50 * Pi / 180, 0});
    problem.setStateInfo(
            "/jointset/knee_r/knee_angle_r/value", {-50 * Pi / 180, 0});
    problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",
            {-15 * Pi / 180, 25 * Pi / 180});
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
            {-15 * Pi / 180, 25 * Pi / 180});
    problem.setStateInfo("/jointset/lumbar/lumbar/value", {0, 20 * Pi / 180});

    // Configure the solver.
    // =====================
    MocoCasADiSolver& solver = study.updSolver<MocoCasADiSolver>();
    solver.set_num_mesh_intervals(50);
    solver.set_verbosity(2);
    solver.set_optim_solver("ipopt");
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(10000);

    // Solve problem.
    // ==============
    MocoSolution solution = study.solve();
    auto full = createPeriodicTrajectory(solution);
    full.write("gaitTrackingMetabolics_solution_fullcycle.sto");
    std::cout << "The metabolic cost of transport is: "
        << solution.getObjectiveTerm("met") << " [J kg-1 m-1]." << std::endl;
    study.visualize(solution);

}

int main() {
    try {
        gaitTrackingMetabolics();
    } catch (const std::exception& e) { std::cout << e.what() << std::endl; }
    return EXIT_SUCCESS;
}
