/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxMocoUserControlCost.cpp                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Prasanna Sritharan, Nicholas Bianco                             *
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
#include <OpenSim/OpenSim.h>
#include "MocoUserControlCost.h"

using namespace OpenSim;

// user-defined constraint function (currently same as MocoConstraintCost)
double my_constraint_cost_function(const SimTK::State& state,
        const Model& model, std::vector<double> utility,
        std::vector<double> ctrl_weights, std::vector<int> ctrl_indices) {
    double integrand = 0;

    model.realizeVelocity(state); // TODO would avoid this, ideally.
    const auto& controls = model.getControls(state);

    int iweight = 0;
    for (const auto& icontrol : ctrl_indices) {
        integrand +=
                ctrl_weights[iweight] * controls[icontrol] * controls[icontrol];
        ++iweight;
    }

    return integrand;
}

/// Convenience function to apply an CoordinateActuator to the model.
void addCoordinateActuator(std::unique_ptr<Model>& model, std::string coordName,
        double optimalForce) {

    auto& coordSet = model->updCoordinateSet();

    auto* actu = new CoordinateActuator();
    actu->setName("tau_" + coordName);
    actu->setCoordinate(&coordSet.get(coordName));
    actu->setOptimalForce(optimalForce);
    actu->setMinControl(-1);
    actu->setMaxControl(1);
    model->addComponent(actu);
}

int main() {

    // create a new MocoStudy
    MocoStudy study;
    study.setName("my_study");

    // PREPARE THE MODEL
    // ----------------------------------

    // get the base model
    auto model = make_unique<Model>("subject01.osim");

    // add coordinate actuators
    addCoordinateActuator(model, "lumbar_extension", 500);
    addCoordinateActuator(model, "pelvis_tilt", 500);
    addCoordinateActuator(model, "pelvis_tx", 1000);
    addCoordinateActuator(model, "pelvis_ty", 2500);
    addCoordinateActuator(model, "hip_flexion_r", 100);
    addCoordinateActuator(model, "knee_angle_r", 100);
    addCoordinateActuator(model, "ankle_angle_r", 100);
    addCoordinateActuator(model, "hip_flexion_l", 100);
    addCoordinateActuator(model, "knee_angle_l", 100);
    addCoordinateActuator(model, "ankle_angle_l", 100);

    // DEFINE THE OPTIMAL CONTROL PROBLEM
    // ----------------------------------

    // get the MocoProblem
    MocoProblem& problem = study.updProblem();

    // set the model
    problem.setModelCopy(*model);

    // set time bounds
    problem.setTimeBounds(MocoInitialBounds(0.0), MocoFinalBounds(2.5));

    // set tracking cost
    auto* trackingcost = problem.addCost<MocoMarkerTrackingCost>();
    trackingcost->setName("marker_cost");

    // create a markers reference object
    MarkersReference markerref("marker_trajectories.trc");

    // set marker tracking weights
    Set<MarkerWeight> markerweights;
    markerweights.cloneAndAppend({"Top.Head", 3});
    markerweights.cloneAndAppend({"R.ASIS", 3});
    markerweights.cloneAndAppend({"L.ASIS", 3});
    markerweights.cloneAndAppend({"V.Sacral", 3});
    markerweights.cloneAndAppend({"R.Heel", 2});
    markerweights.cloneAndAppend({"R.Toe.Tip", 2});
    markerweights.cloneAndAppend({"L.Heel", 2});
    markerweights.cloneAndAppend({"L.Toe.Tip", 2});

    // update the marker weights
    markerref.setMarkerWeightSet(markerweights);

    // apply the marker trajectories and weights to the problem
    trackingcost->setMarkersReference(markerref);

    // allow unused references in TRC file
    trackingcost->setAllowUnusedReferences(true);

    // create a user-defined control cost
    auto* controlcost = problem.addCost<MocoUserControlCost>();
    controlcost->setName("my_control_cost");
    controlcost->set_weight(0.1);

    // pass in the control cost function
    // (no utility parameters in this example)
    controlcost->user_control_cost_fun_ptr = my_constraint_cost_function;

    // SOLVE OPTIMAL CONTROL PROBLEM
    // ----------------------------------

    // get the NLP solver
    MocoCasADiSolver& solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(20);
    solver.set_optim_constraint_tolerance(1e-3);
    solver.set_optim_convergence_tolerance(1e-3);

    // set initial guess
    solver.setGuess("bounds"); // not sure what this does

    // solve and write to file
    MocoSolution solution = study.solve();
    solution.write("sandboxMocoUserControlCost_marker_tracking_solution.sto");

    // visualise solution
    study.visualize(solution);

    return EXIT_SUCCESS;
}
