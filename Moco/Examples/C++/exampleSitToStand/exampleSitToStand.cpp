/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleSitToStand.cpp                                        *
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
//#include <OpenSim/Common/osimCommon.h>
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

    Model model("sitToStand_3dof9musc.osim");

    removeMuscles(model);
    //addCoordinateActuator(model, "hip_flexion_r", 500);
    //addCoordinateActuator(model, "knee_angle_r", 500);
    //addCoordinateActuator(model, "ankle_angle_r", 500);

    return model;
}

MocoTool configureMocoTool() {

    // Create a MocoTool instance.
    MocoTool moco;

    // Get the empty MocoSolver (here MocoCasADiSolver) from the MocoTool.
    auto& solver = moco.initCasADiSolver();
    solver.set_num_mesh_points(25);
    solver.set_dynamics_mode("implicit"); // default: "explicit"
    solver.set_optim_convergence_tolerance(1e-2);
    solver.set_optim_constraint_tolerance(1e-2);
    solver.set_transcription_scheme("hermite-simpson");
    solver.set_enforce_constraint_derivatives(true);
    solver.set_optim_hessian_approximation("limited-memory");

    // Get the empty MocoProblem from the MocoTool.
    auto& problem = moco.updProblem();
    // Set the bounds for the MocoProblem.
    problem.setTimeBounds(0, 1);
    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",
        MocoBounds(-2, 0.5), MocoInitialBounds(-2), MocoFinalBounds(0));
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",
        {-2, 0}, -2, 0);
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value", 
        {-0.5, 0.7}, -0.5, 0);

    problem.setStateInfo("/jointset/hip_r/hip_flexion_r/speed",
        {-50, 50}, 0, 0);
    problem.setStateInfo("/jointset/knee_r/knee_angle_r/speed",
        {-50, 50}, 0, 0);   
    problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/speed",
        {-50, 50}, 0, 0);

    return moco;
}

void main() {

    MocoTool moco = configureMocoTool();

    auto& problem = moco.updProblem();
    problem.setModelCopy(createTorqueDrivenModel());
    problem.addCost<MocoControlCost>();

    auto& solver = moco.updSolver<MocoCasADiSolver>();
    solver.resetProblem(problem);
    solver.createGuess("bounds");

    MocoSolution solution = moco.solve();
    moco.visualize(solution);

}