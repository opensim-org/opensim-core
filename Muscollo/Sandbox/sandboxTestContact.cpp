/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: sandboxTestContact.cpp                                   *
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

// This file will become testContact.

// TODO add 3D tests (contact models are currently only 2D).

#include <Muscollo/osimMuscollo.h>
#include <OpenSim/Simulation/Manager/Manager.h>

#include "MuscolloSandboxShared.h"

const double FRICTION_COEFFICIENT = 0.7;

using namespace OpenSim;
using SimTK::Vec3;

using CreateContactFunction = std::function<StationPlaneContactForce*(void)>;
Model create2DPointMassModel(CreateContactFunction createContact) {
    Model model;
    model.setName("point_mass");
    auto* intermed = new Body("intermed", 0, Vec3(0), SimTK::Inertia(0));
    model.addComponent(intermed);
    auto* body = new Body("body", 50.0, Vec3(0), SimTK::Inertia(1));
    model.addComponent(body);

    // Allows translation along x.
    auto* jointX = new SliderJoint();
    jointX->setName("tx");
    jointX->connectSocket_parent_frame(model.getGround());
    jointX->connectSocket_child_frame(*intermed);
    auto& coordX = jointX->updCoordinate(SliderJoint::Coord::TranslationX);
    coordX.setName("tx");
    model.addComponent(jointX);

    // The joint's x axis must point in the global "+y" direction.
    auto* jointY = new SliderJoint("ty",
            *intermed, Vec3(0), Vec3(0, 0, 0.5 * SimTK::Pi),
            *body, Vec3(0), Vec3(0, 0, 0.5 * SimTK::Pi));
    auto& coordY = jointY->updCoordinate(SliderJoint::Coord::TranslationX);
    coordY.setName("ty");
    model.addComponent(jointY);

    auto* station = new Station();
    station->setName("contact_point");
    station->connectSocket_parent_frame(*body);
    model.addComponent(station);

    auto* force = createContact();
    force->setName("contact");
    model.addComponent(force);
    force->connectSocket_station(*station);

    return model;
}

// Test that, with dissipation, the contact force settles to the weight of the
// system. This is the same type of test done in OpenSim's testForces for
// HuntCrossleyForce. The test is performed with both time stepping and direct
// collocation.
SimTK::Real testNormalForce(CreateContactFunction createContact) {
    Model model(create2DPointMassModel(createContact));
    SimTK::Real weight;
    {
        SimTK::State state = model.initSystem();
        weight = model.getTotalMass(state) * (-model.getGravity()[1]);
    }

    const SimTK::Real y0 = 0.5;
    const SimTK::Real finalTime = 2.0;

    // Time stepping.
    // --------------
    SimTK::Real finalHeightTimeStepping;
    {
        SimTK::State state = model.initSystem();
        model.setStateVariableValue(state, "ty/ty/value", y0);
        SimTK::RungeKuttaMersonIntegrator integ(model.getMultibodySystem());
        integ.setAccuracy(1e-6);
        Manager manager(model, state, integ);
        state = manager.integrate(finalTime);

        // visualize(model, manager.getStateStorage());

        auto& contact = model.getComponent<StationPlaneContactForce>("contact");
        model.realizeVelocity(state);
        const Vec3 contactForce = contact.calcContactForceOnStation(state);
        // The horizontal force is not quite zero, maybe from a buildup of
        // numerical error (tightening the accuracy reduces this force).
        SimTK_TEST_EQ_TOL(contactForce[0], 0, 0.01);
        SimTK_TEST_EQ_TOL(contactForce[1], weight, 0.01);
        // The system is planar, so there is no force in the z direction.
        SimTK_TEST_EQ(contactForce[2], 0);

        finalHeightTimeStepping =
                model.getStateVariableValue(state, "ty/ty/value");
    }

    // Direct collocation.
    // -------------------
    // This is a simulation (initial value problem), not a trajectory
    // optimization.
    SimTK::Real finalHeightDircol;
    {
        MucoTool muco;
        MucoProblem& mp = muco.updProblem();
        mp.setModel(model);
        mp.setTimeBounds(0, finalTime);
        mp.setStateInfo("tx/tx/value", {-5, 5}, 0);
        mp.setStateInfo("ty/ty/value", {-0.5, 1}, y0);
        mp.setStateInfo("tx/tx/speed", {-10, 10}, 0);
        mp.setStateInfo("ty/ty/speed", {-10, 10}, 0);

        MucoTropterSolver& ms = muco.initSolver();
        ms.set_num_mesh_points(50);

        MucoSolution solution = muco.solve();
        // muco.visualize(solution);

        auto statesTraj = solution.exportToStatesTrajectory(mp);
        const auto& finalState = statesTraj.back();
        model.realizeVelocity(finalState);
        auto& contact = model.getComponent<StationPlaneContactForce>("contact");
        const Vec3 contactForce = contact.calcContactForceOnStation(finalState);
        // For some reason, direct collocation doesn't produce the same
        // numerical issues with the x component of the force as seen above.
        SimTK_TEST_EQ(contactForce[0], 0);
        SimTK_TEST_EQ_TOL(contactForce[1], weight, 0.01);
        SimTK_TEST_EQ(contactForce[2], 0);

        finalHeightDircol =
                model.getStateVariableValue(finalState, "ty/ty/value");
    }

    SimTK_TEST_EQ_TOL(finalHeightTimeStepping, finalHeightDircol, 1e-5);

    return finalHeightTimeStepping;
}

// Test the friction component of the contact force by ensuring that the point
// mass travels the expected horizontal distance if it starts in the ground.
// To make the friction force roughly constant, we want the equilibruim height
// of the mass (from testNormalForce()).
void testFrictionForce(CreateContactFunction createContact,
        const SimTK::Real& equilibriumHeight) {
    Model model(create2DPointMassModel(createContact));
    SimTK::Real weight;
    {
        SimTK::State state = model.initSystem();
        weight = model.getTotalMass(state) * (-model.getGravity()[1]);
    }

    const SimTK::Real y0 = equilibriumHeight;
    const SimTK::Real finalTime = 0.5;
    const SimTK::Real vx0 = 2.5;

    const SimTK::Real g = -model.getGravity()[1];

    // Expected final x position.
    // --------------------------
    // m * vxdot = F = - mu * m * g
    // vx(t) = -mu * g * t + vx0
    // x(t) = -0.5 * mu * g * t^2 + vx0 * t
    // The time at which the point reaches rest:
    // t_rest = vx0 / (mu * g)
    // Final position: x(t_rest)
    const double& mu = FRICTION_COEFFICIENT;
    const SimTK::Real restTime = vx0 / (mu * g);
    assert(restTime < finalTime);
    const SimTK::Real expectedFinalX =
            -0.5 * mu * g * pow(restTime, 2) + vx0 * restTime;

    // Time stepping.
    // --------------
    {
        SimTK::State state = model.initSystem();
        model.setStateVariableValue(state, "ty/ty/value", y0);
        model.setStateVariableValue(state, "tx/tx/speed", vx0);
        Manager manager(model, state);
        state = manager.integrate(finalTime);

        // visualize(model, manager.getStateStorage());

        const SimTK::Real finalTX =
                model.getStateVariableValue(state, "tx/tx/value");
        SimTK_TEST_EQ_TOL(finalTX, expectedFinalX, 0.005);

        // The system should be at rest.
        SimTK_TEST_EQ_TOL(state.getU(),
                SimTK::Vector(state.getNU(), 0.0), 1e-3);

    }

    // Direct collocation.
    // -------------------
    // This is a simulation (initial value problem), not a trajectory
    // optimization.
    SimTK::Real finalHeightDircol;
    {
        MucoTool muco;
        MucoProblem& mp = muco.updProblem();
        mp.setModel(model);
        mp.setTimeBounds(0, finalTime);
        mp.setStateInfo("tx/tx/value", {-5, 5}, 0);
        mp.setStateInfo("ty/ty/value", {-0.5, 1}, y0);
        mp.setStateInfo("tx/tx/speed", {-10, 10}, vx0);
        mp.setStateInfo("ty/ty/speed", {-10, 10}, 0);

        MucoTropterSolver& ms = muco.initSolver();
        ms.set_num_mesh_points(25);

        MucoSolution solution = muco.solve();
        // muco.visualize(solution);

        auto statesTraj = solution.exportToStatesTrajectory(mp);
        const auto& finalState = statesTraj.back();
        const SimTK::Real finalTX =
                model.getStateVariableValue(finalState, "tx/tx/value");

        SimTK_TEST_EQ_TOL(finalTX, expectedFinalX, 0.005);

        // The system should be at rest.
        SimTK_TEST_EQ_TOL(finalState.getU(),
                SimTK::Vector(finalState.getNU(), 0.0), 1e-3);
    }
}

void testStationPlaneContactForce(CreateContactFunction createContact) {
    const SimTK::Real equilibriumHeight = testNormalForce(createContact);
    testFrictionForce(createContact, equilibriumHeight);
}


AckermannVanDenBogert2010Force* createAVDB() {
    auto* contact = new AckermannVanDenBogert2010Force();
    contact->set_stiffness(1e5);
    contact->set_dissipation(1.0);
    contact->set_friction_coefficient(FRICTION_COEFFICIENT);
    return contact;
}

EspositoMiller2018Force* createEspositoMiller() {
    auto* contact = new EspositoMiller2018Force();
    contact->set_stiffness(1e5);
    contact->set_dissipation(1.0);
    contact->set_friction_coefficient(FRICTION_COEFFICIENT);
    return contact;
}

MeyerFregly2016Force* createMeyerFregly() {
    auto* contact = new MeyerFregly2016Force();
    contact->set_stiffness(1e5);
    contact->set_dissipation(1.0);
    // TODO set friction coefficient.
    return contact;
}

int main() {
    SimTK_START_TEST("testContact");
        SimTK_SUBTEST1(testStationPlaneContactForce, createAVDB);
        SimTK_SUBTEST1(testStationPlaneContactForce, createEspositoMiller);
        // TODO does not pass:
        // SimTK_SUBTEST1(testStationPlaneContactForce, createMeyerFregly);
    SimTK_END_TEST();
}
