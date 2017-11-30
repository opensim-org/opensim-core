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
    int N = 10;
    MucoSolution sol1;
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
        ms.set_num_mesh_points(N);

        sol1 = muco.solve();
        sol1.write("testMucoCosts_testMucoControlCost_sol1.sto");

        // Minimum effort solution is a linear control.
        SimTK_TEST_EQ_TOL(sol1.getControl("actuator"),
                createVectorLinspace(N, 2.23, -2.23), 0.25);
        // Symmetry.
        SimTK_TEST_EQ_TOL(sol1.getControl("actuator").getElt(0, 0),
                -sol1.getControl("actuator").getElt(N-1, 0), 1e-5);

        // Minimum effort solution takes as long as possible.
        SimTK_TEST_EQ(sol1.getTime().getElt(N-1, 0), 5);
    }

    // TODO test that we can ignore specific actuators.
    // TODO for now, the weight can just be set to 0 (not ideal).
    //{

    //    MucoTool muco;
    //    muco.setName("sliding_mass");
    //    MucoProblem& mp = muco.updProblem();
    //    MucoProblem& mp = muco.updProblem();
    //    auto model = createSlidingMassModel();

    //    auto* actu = new CoordinateActuator();
    //    actu->setCoordinate(&model.getCoordinateSet().get("position"));
    //    actu->setName("actuator2");
    //    actu->setOptimalForce(1);
    //    model.addComponent(actu);

    //    mp.setModel(model);
    //    mp.setTimeBounds(0, 5);
    //    mp.setStateInfo("slider/position/value", {0, 1}, 0, 1);
    //    mp.setStateInfo("slider/position/speed", {-100, 100}, 0, 0);
    //    mp.setControlInfo("actuator", MucoBounds(-10, 10));
    //    mp.setControlInfo("actuator2", MucoBounds(-10, 10));

    //    MucoControlCost effort;
    //    effort.addActuatorToInclude("actuator");
    //    effort.set
    //    mp.addCost(effort);

    //    MucoTropterSolver& ms = muco.initSolver();
    //    ms.set_num_mesh_points(N);

    //    MucoSolution solution = muco.solve();
    //    SimTK_TEST_EQ(solution.getControl("actuator2"), SimTK::Vector(N, 0));
    //}

    // TODO test from XML.

    // Ensure that the weights cause one actuator to be preferred over
    // another.
    MucoSolution sol2;
    std::string omucoFile = "testMucoCosts_testMucoControlCost.omuco";
    {
        MucoTool muco;
        muco.setName("sliding_mass");
        muco.set_write_solution("false");
        MucoProblem& mp = muco.updProblem();
        auto model = createSlidingMassModel();

        auto* actu = new CoordinateActuator();
        actu->setCoordinate(&model.updCoordinateSet().get("position"));
        actu->setName("actuator2");
        actu->setOptimalForce(1);
        model.addComponent(actu);

        mp.setModel(model);
        mp.setTimeBounds(0, 5);
        mp.setStateInfo("slider/position/value", {0, 1}, 0, 1);
        mp.setStateInfo("slider/position/speed", {-100, 100}, 0, 0);
        mp.setControlInfo("actuator", MucoBounds(-10, 10));
        mp.setControlInfo("actuator2", MucoBounds(-10, 10));

        MucoControlCost effort;
        effort.setWeight("actuator2", 2.0);
        mp.addCost(effort);

        MucoTropterSolver& ms = muco.initSolver();
        ms.set_num_mesh_points(N);

        sol2 = muco.solve();

        muco.print(omucoFile);

        // The actuator with the lower weight is more active.
        SimTK_TEST_EQ_TOL(sol2.getControl("actuator"),
                2 * sol2.getControl("actuator2"), 1e-5);
        // Sum of control for these two actuators is the same as the control
        // in the single-actuator case.
        SimTK_TEST_EQ_TOL(sol2.getControlsTrajectory().rowSum(),
            sol1.getControl("actuator"), 1e-5);
    }

    // Cannot set a weight for a nonexistant control.
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
        effort.setWeight("nonexistant", 1.5);
        mp.addCost(effort);
        MucoTropterSolver& ms = muco.initSolver();
        ms.set_num_mesh_points(8);
        ms.set_optim_max_iterations(1);
        SimTK_TEST_MUST_THROW_EXC(muco.solve(), Exception);
    }

    // De/serialization.
    {
        MucoTool muco(omucoFile);
        MucoSolution solDeserialized = muco.solve();
        sol2.write("DEBUG_sol2.sto");
        solDeserialized.write("DEBUG_solDeserialized.sto");
        SimTK_TEST(solDeserialized.isNumericallyEqual(sol2, 1e-5));
    }
}

int main() {
    SimTK_START_TEST("testMucoCosts");
        SimTK_SUBTEST(testMucoControlCost);
    SimTK_END_TEST();
}
