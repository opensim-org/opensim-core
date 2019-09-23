/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoAnalytic.cpp                                         *
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

#define CATCH_CONFIG_MAIN
#include "Testing.h"
#include <Moco/osimMoco.h>

#include <OpenSim/Actuators/SpringGeneralizedForce.h>

using namespace OpenSim;

/// Taken from Kirk 1998 equations 5.1-69 and 5.1-70, page 199.
SimTK::Matrix expectedSolution(const SimTK::Vector& time) {
    using std::exp;
    SimTK::Mat22 A(-2 - 0.5 * exp(-2) + 0.5 * exp(2),
            1 - 0.5 * exp(-2) - 0.5 * exp(2), -1 + 0.5 * exp(-2) + 0.5 * exp(2),
            0.5 * exp(-2) - 0.5 * exp(2));
    SimTK::Vec2 b(5, 2);
    SimTK::Vec2 c = A.invert() * b;
    const double c2 = c[0];
    const double c3 = c[1];

    auto x0_func = [&c2, &c3](const double& t) -> double {
        return c2 * (-t - 0.5 * exp(-t) + 0.5 * exp(t)) +
               c3 * (1 - 0.5 * exp(-t) - 0.5 * exp(t));
    };

    auto x1_func = [&c2, &c3](const double& t) -> double {
        return c2 * (-1 + 0.5 * exp(-t) + 0.5 * exp(t)) +
               c3 * (0.5 * exp(-t) - 0.5 * exp(t));
    };

    SimTK::Matrix expectedStatesTrajectory(time.size(), 2);
    for (int itime = 0; itime < time.size(); ++itime) {
        expectedStatesTrajectory(itime, 0) = x0_func(time[itime]);
        expectedStatesTrajectory(itime, 1) = x1_func(time[itime]);
    }
    return expectedStatesTrajectory;
}

TEMPLATE_TEST_CASE("Second order linear min effort", "", MocoTropterSolver,
        MocoCasADiSolver) {
    // Kirk 1998, Example 5.1-1, page 198.

    Model model;
    auto* body = new Body("b", 1, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addBody(body);

    auto* joint = new SliderJoint("j", model.getGround(), *body);
    joint->updCoordinate().setName("coord");
    model.addJoint(joint);

    auto* damper = new SpringGeneralizedForce("coord");
    damper->setViscosity(-1.0);
    model.addForce(damper);

    auto* actu = new CoordinateActuator("coord");
    model.addForce(actu);
    model.finalizeConnections();

    MocoStudy moco;
    auto& problem = moco.updProblem();

    problem.setModelCopy(model);
    problem.setTimeBounds(0, 2);
    problem.setStateInfo("/jointset/j/coord/value", {-10, 10}, 0, 5);
    problem.setStateInfo("/jointset/j/coord/speed", {-10, 10}, 0, 2);
    problem.setControlInfo("/forceset/coordinateactuator", {-50, 50});

    problem.addGoal<MocoControlGoal>("effort", 0.5);

    auto& solver = moco.initSolver<TestType>();
    solver.set_num_mesh_intervals(50);
    MocoSolution solution = moco.solve();

    const auto expected = expectedSolution(solution.getTime());

    OpenSim_CHECK_MATRIX_ABSTOL(solution.getStatesTrajectory(), expected, 1e-5);
}
