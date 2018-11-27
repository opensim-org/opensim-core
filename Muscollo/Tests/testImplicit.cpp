/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: testImplicit.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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
#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <Muscollo/osimMuscollo.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>

using namespace OpenSim;

MucoSolution solveDoublePendulumSwingup(const std::string& dynamics_mode) {
    MucoTool muco;
    muco.setName("double_pendulum_swingup_" + dynamics_mode);

    // Define the optimal control problem.
    // ===================================
    MucoProblem& mp = muco.updProblem();

    // Model (dynamics).
    // -----------------
    mp.setModelCopy(ModelFactory::createDoublePendulumModel());

    // Bounds.
    // -------
    mp.setTimeBounds(0, {0, 5});
    mp.setStateInfo("/jointset/j0/q0/value", {-10, 10}, 0);
    mp.setStateInfo("/jointset/j0/q0/speed", {-50, 50}, 0, 0);
    mp.setStateInfo("/jointset/j1/q1/value", {-10, 10}, 0);
    mp.setStateInfo("/jointset/j1/q1/speed", {-50, 50}, 0, 0);
    mp.setControlInfo("/tau0", {-100, 100}); // TODO tighten.
    mp.setControlInfo("/tau1", {-100, 100});

    // Cost.
    // -----
    auto* ftCost = mp.addCost<MucoFinalTimeCost>();
    ftCost->set_weight(0.001);

    auto* endpointCost = mp.addCost<MucoMarkerEndpointCost>("endpoint");
    endpointCost->set_weight(1000.0);
    endpointCost->setPointName("/markerset/marker");
    endpointCost->setReferenceLocation(SimTK::Vec3(0, 2, 0));

    // Configure the solver.
    // =====================
    int N = 30;
    MucoTropterSolver& solver = muco.initSolver();
    solver.set_dynamics_mode(dynamics_mode);
    solver.set_num_mesh_points(N);
    //solver.set_verbosity(2);
    //solver.set_optim_hessian_approximation("exact");

    MucoIterate guess = solver.createGuess();
    guess.setNumTimes(2);
    guess.setTime({0, 1});
    guess.setState("/jointset/j0/q0/value", {0, -SimTK::Pi});
    guess.setState("/jointset/j1/q1/value", {0, 2*SimTK::Pi});
    guess.setState("/jointset/j0/q0/speed", {0, 0});
    guess.setState("/jointset/j1/q1/speed", {0, 0});
    guess.setControl("/tau0", {0, 0});
    guess.setControl("/tau1", {0, 0});
    guess.resampleWithNumTimes(10);
    solver.setGuess(guess);

    // muco.visualize(guess);

    muco.print("double_pendulum_swingup_" + dynamics_mode + ".omuco");

    // Solve the problem.
    // ==================
    MucoSolution solution = muco.solve();
    // muco.visualize(solution);

    return solution;

}

SCENARIO("Similar solutions between implicit and explicit dynamics modes",
        "[implicit]") {
    GIVEN("solutions to implicit and explicit problems") {

        auto solutionImplicit = solveDoublePendulumSwingup("implicit");
        // TODO: Solving the implicit problem multiple times gives different
        // results; https://github.com/stanfordnmbl/moco/issues/172.
        // auto solutionImplicit2 = solveDoublePendulumSwingup("implicit");
        // TODO: The solution to this explicit problem changes every time.
        auto solution = solveDoublePendulumSwingup("explicit");

        CAPTURE(solutionImplicit.getFinalTime(), solution.getFinalTime());

        const double stateError =
                solutionImplicit.compareContinuousVariablesRMS(solution,
                        {}, /* controlNames: */ {"none"}, {"none"},
                        /* derivativeNames: */ {"none"});

        // There is more deviation in the controls.
        const double controlError =
                solutionImplicit.compareContinuousVariablesRMS(solution,
                        {"none"}, /* controlNames: */ {}, {"none"}, {"none"});

        CAPTURE(stateError, controlError);

        THEN("solutions are approximately equal") {
            REQUIRE(solutionImplicit.getFinalTime() ==
                    Approx(solution.getFinalTime()).margin(1e-2));
            REQUIRE(stateError < 2.0);
            REQUIRE(controlError < 30.0);
        }
    }
}
// TODO compareContinuousVariables: handle derivatives.
// TODO test reading/writing MucoIterate with derivatives.
