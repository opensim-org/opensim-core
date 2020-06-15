/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSquatToStand.cpp                                      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

#include <Moco/osimMoco.h>

#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Simulation/osimSimulation.h>

using namespace OpenSim;

/// Convenience function to apply an CoordinateActuator to the model.
void addCoordinateActuator(Model& model, std::string coordName,
        double optimalForce) {

    auto& coordSet = model.updCoordinateSet();

    auto* actu = new CoordinateActuator();
    actu->setName("tau_" + coordName);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(optimalForce);
    actu->setMinControl(-1);
    actu->setMaxControl(1);
    model.addComponent(actu);
}

Model createTorqueDrivenModel() {

    Model model("squatToStand_3dof9musc.osim");

    ModelFactory::removeMuscles(model);
    addCoordinateActuator(model, "hip_flexion_r", 100);
    addCoordinateActuator(model, "knee_angle_r", 300);
    addCoordinateActuator(model, "ankle_angle_r", 100);

    return model;
}

Model createMuscleDrivenModel() {

    Model model("squatToStand_3dof9musc.osim");

    model.finalizeConnections();
    DeGrooteFregly2016Muscle::replaceMuscles(model);
    for (int m = 0; m < model.getMuscles().getSize(); ++m) {
        auto& musc = model.updMuscles().get(m);
        musc.set_ignore_activation_dynamics(true);
        musc.set_ignore_tendon_compliance(true);
        musc.set_max_isometric_force(10*musc.get_max_isometric_force());
    }

    //addCoordinateActuator(model, "hip_flexion_r", 50);
    //addCoordinateActuator(model, "knee_angle_r", 100);
    //addCoordinateActuator(model, "ankle_angle_r", 50);
    return model;
}

MocoStudy configureMocoStudy() {

    // Create a MocoStudy instance.
    MocoStudy study;

    // Get the empty MocoSolver (here MocoCasADiSolver) from the MocoStudy.
    auto& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(25);
    solver.set_multibody_dynamics_mode("implicit"); // default: "explicit"
    solver.set_optim_convergence_tolerance(1e-6);
    solver.set_optim_constraint_tolerance(1e-6);
    solver.set_transcription_scheme("hermite-simpson");
    solver.set_enforce_constraint_derivatives(true);
    solver.set_optim_hessian_approximation("limited-memory");
    solver.set_optim_finite_difference_scheme("forward");

    // Get the empty MocoProblem from the MocoStudy.
    auto& problem = study.updProblem();
    // Set the bounds for the MocoProblem.
    problem.setTimeBounds(0, 1);
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
            MocoBounds(-2, 0.5), MocoInitialBounds(-2), MocoFinalBounds(0));
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
            {-2, 0}, -2, 0);
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",
            {-0.5, 0.7}, -0.5, 0);

    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/speed",
            {-50, 50}, 0, {-50, 50});
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/speed",
            {-50, 50}, 0, {-50, 50});
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/speed",
            {-50, 50}, 0, {-50, 50});

    return study;
}

int main() {

    //MocoStudy study = configureMocoStudy();
    //auto& problem = study.updProblem();
    //problem.setModelCopy(createTorqueDrivenModel());
    //problem.addCost<MocoControlCost>();

    //auto& solver = study.updSolver<MocoCasADiSolver>();
    //solver.resetProblem(problem);
    //solver.createGuess("bounds");

    //MocoSolution predictSolution = study.solve();
    //predictSolution.write("predictSolution.sto");
    //study.visualize(predictSolution);


    //MocoStudy mocoTracking = configureMocoStudy();
    //auto& problemTracking = mocoTracking.updProblem();
    //problemTracking.setModelCopy(createTorqueDrivenModel());

    //auto* tracking = problemTracking.addCost<MocoStateTrackingCost>();
    //tracking->setName("tracking");
    //tracking->setReferenceFile("predictSolution.sto");
    //tracking->setAllowUnusedReferences(true);
    //tracking->setWeight("/jointset/patellofemoral_r/knee_angle_r_beta/value", 0);
    //tracking->setWeight("/jointset/patellofemoral_r/knee_angle_r_beta/value", 0);

    //auto& solverTracking = mocoTracking.updSolver<MocoCasADiSolver>();
    //solverTracking.resetProblem(problemTracking);
    //solverTracking.createGuess("bounds");

    //MocoSolution trackingSolution = mocoTracking.solve();
    //trackingSolution.write("trackingSolution.sto");
    //mocoTracking.visualize(trackingSolution);


    MocoStudy mocoMusclePredict = configureMocoStudy();
    auto& problemMusclePredict = mocoMusclePredict.updProblem();
    problemMusclePredict.setModelCopy(createMuscleDrivenModel());

    auto& solverMusclePredict = mocoMusclePredict.updSolver<MocoCasADiSolver>();
    solverMusclePredict.resetProblem(problemMusclePredict);
    solverMusclePredict.createGuess("bounds");

    MocoSolution musclePredictSolutionNoCost = mocoMusclePredict.solve();


    problemMusclePredict.addGoal<MocoControlGoal>();
    solverMusclePredict.resetProblem(problemMusclePredict);
    solverMusclePredict.setGuess(musclePredictSolutionNoCost);


    MocoSolution musclePredictSolution = mocoMusclePredict.solve();
    musclePredictSolution.write("musclePredictSolution.sto");
    mocoMusclePredict.visualize(musclePredictSolution);

    return EXIT_SUCCESS;
}

