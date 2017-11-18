/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: testMucoCosts.cpp                                        *
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

#include <Muscollo/osimMuscollo.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Actuators/CoordinateActuator.h>

using namespace OpenSim;

Model createSlidingMassModel() {
    Model model;
    model.setName("sliding_mass");
    model.set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 10.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model.addComponent(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    model.addComponent(actu);

    return model;
}

/// Test the result of a sliding mass minimum effort problem.
void testMucoControlCost() {
    {
        MucoTool muco;
        muco.setName("sliding_mass");
        MucoProblem& mp = muco.updProblem();
        mp.setModel(createSlidingMassModel());
        mp.setTimeBounds(0, {0, 5});
        mp.setStateInfo("slider/position/value", {0, 1}, 0, 1);
        mp.setStateInfo("slider/position/speed", {-100, 100}, 0, 0);
        mp.setControlInfo("actuator", MucoBounds(-10, 10));

        MucoControlCost effort;
        mp.addCost(effort);

        MucoTropterSolver& ms = muco.initSolver();
        int N = 10;
        ms.set_num_mesh_points(10);

        MucoSolution solution = muco.solve();
        solution.write("testMucoCosts_testMucoControlCost_solution.sto");

        // TODO add checks here.
        // Minimum effort solution is a linear control.
        SimTK_TEST_EQ_TOL(solution.getControl("actuator"),
                createVectorLinspace(N, 2.23, -2.23), 0.25);
        // Symmetry.
        SimTK_TEST_EQ_TOL(solution.getControl("actuator").getElt(0, 0),
                -solution.getControl("actuator").getElt(N-1, 0), 1e-5);

        // Minimum effort solution takes as long as possible.
        SimTK_TEST_EQ(solution.getTime().getElt(N-1, 0), 5);
    }

    // TODO test that we can ignore specific actuators.

    // Ensure that the weights cause one actuator to be preferred over
    // another.
    /*{
        MucoTool muco;
        muco.setName("sliding_mass");
        muco.set_write_solution("false");
        MucoProblem& mp = muco.updProblem();
        auto model = createSlidingMassModel();

        auto* actu = new CoordinateActuator();
        actu->setCoordinate(model.getCoordinate("position"));
        actu->setName("actuator2");
        actu->setOptimalForce(1);
        model.addComponent(actu);

        mp.setModel(model);
        mp.setTimeBounds(0, 1);
        mp.setStateInfo("slider/position/value", {0, 1}, 0, 1);
        mp.setStateInfo("slider/position/speed", {-100, 100}, 0, 0);
        mp.setControlInfo("actuator", MucoBounds(-10, 10));
        mp.setControlInfo("actuator2", MucoBounds(-10, 10));

        MucoControlCost effort;
        effort.setWeight("actuator2", 2.0);
        mp.addCost(effort);

        MucoTropterSolver& ms = muco.initSolver();
        ms.set_num_mesh_points(20);

        MucoSolution sol = muco.solve();

        // The actuator with the lower weight is more active.
        SimTK_TEST_EQ(sol.getControl("actuator"),
                2 * sol.getControl("actuator2"));
    }*/
}

int main() {
    SimTK_START_TEST("testMucoCosts");
        SimTK_SUBTEST(testMucoControlCost);
    SimTK_END_TEST();
}
