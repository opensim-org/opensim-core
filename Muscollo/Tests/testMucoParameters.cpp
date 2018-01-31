/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: testMucoParameters.cpp                                   *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include <Muscollo/osimMuscollo.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Actuators/SpringGeneralizedForce.h>
#include <OpenSim/Actuators/CoordinateActuator.h>


using namespace OpenSim;

const double STIFFNESS = 100.0; // N/m
const double MASS = 5.0; // kg
const double FINAL_TIME = SimTK::Pi * sqrt(MASS / STIFFNESS);

Model createOscillatorModel() {
    Model model;
    model.setName("oscillator");
    model.set_gravity(SimTK::Vec3(0, 0, 0));
    // Set model with incorrect mass value.
    auto* body = new Body("body", 0.5*MASS, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model.addComponent(joint);

    auto* spring = new SpringGeneralizedForce();
    spring->set_coordinate("position");
    spring->setRestLength(0.0);
    spring->setStiffness(STIFFNESS);
    spring->setViscosity(0.0);
    model.addComponent(spring);

    return model;
}

class FinalPositionCost : public MucoCost {
protected:
    void calcEndpointCostImpl(const SimTK::State& finalState,
            SimTK::Real& cost) const override {
       const auto& finalPosition = finalState.getY()[0];

       cost = (finalPosition - 0.5) * (finalPosition - 0.5);
    }
};

void testOscillatorMass() {
    int N = 100;

    MucoTool muco;
    muco.setName("oscillator_mass");
    MucoProblem& mp = muco.updProblem();
    mp.setModel(createOscillatorModel());
    mp.setTimeBounds(0, FINAL_TIME);
    mp.setStateInfo("slider/position/value", {-5.0, 5.0}, -0.5, {0.25, 0.75});
    mp.setStateInfo("slider/position/speed", {-20, 20}, 0, 0);
    
    MucoParameter mass("oscillator_mass", "body", "mass", MucoBounds(0, 10));
    mp.addParameter(mass);

    FinalPositionCost cost;
    mp.addCost(cost);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(N);

    MucoSolution sol = muco.solve();
    sol.write("testMucoParameters_testOscillatorMass_sol.sto");

    SimTK_TEST_EQ_TOL(sol.getParameter("oscillator_mass"), MASS, 0.0005);
}

Model createOscillatorTwoSpringsModel() {
    Model model;
    model.setName("oscillator_two_springs");
    model.set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", MASS, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model.addComponent(joint);

    auto* spring1 = new SpringGeneralizedForce();
    spring1->setName("spring1");
    spring1->set_coordinate("position");
    spring1->setRestLength(0.0);
    spring1->setStiffness(STIFFNESS);
    spring1->setViscosity(0.0);
    model.addComponent(spring1);

    auto* spring2 = new SpringGeneralizedForce();
    spring2->setName("spring2");
    spring2->set_coordinate("position");
    spring2->setRestLength(0.0);
    spring2->setStiffness(STIFFNESS);
    spring2->setViscosity(0.0);
    model.addComponent(spring2);

    return model;
}

// Optimize the stiffness in two parallel springs and make sure they sum to the 
// equivalent stiffness of a single spring that would produce the same 
// oscillation trajectory.
void testOneParameterTwoSprings() {
    int N = 100;

    MucoTool muco;
    muco.setName("oscillator_spring_stiffnesses");
    MucoProblem& mp = muco.updProblem();
    mp.setModel(createOscillatorTwoSpringsModel());
    mp.setTimeBounds(0, FINAL_TIME);
    mp.setStateInfo("slider/position/value", {-5.0, 5.0}, -0.5, {0.25, 0.75});
    mp.setStateInfo("slider/position/speed", {-20, 20}, 0, 0);

    // Optimize a single stiffness value and apply to both springs.
    std::vector<std::string> components = {"spring1", "spring2"};
    MucoParameter stiffness("spring_stiffness", components, "stiffness", 
        MucoBounds(0, 10));
    mp.addParameter(stiffness);

    FinalPositionCost cost;
    mp.addCost(cost);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(N);

    MucoSolution sol = muco.solve();
    sol.write("testMucoParameters_testOscillatorMassTwoSprings_sol.sto");

    // Since springs add in parallel, both stiffness must be the same value
    // and equal half the original spring stiffness.
    SimTK_TEST_EQ_TOL(sol.getParameter("spring_stiffness"), 0.5*STIFFNESS, 
        0.005);
}

const double Izz = 2; // kg-m^2
const double T = 10; // N-m
const double FINAL_ANG = (T / 2*Izz) * (FINAL_TIME * FINAL_TIME);
const double FINAL_ANGVEL = (T / Izz) * FINAL_TIME;
Model createRotatingBarModel() {
    Model model;
    model.setName("rotating_bar");
    model.set_gravity(SimTK::Vec3(0, 0, 0));
    // Set model with incorrect inertia value.
    auto* body = new Body("body", MASS, SimTK::Vec3(0), 
            SimTK::Inertia(0, 0, 0.5*Izz));
    model.addComponent(body);

    // Allows rotation around z.
    auto* joint = new PinJoint("pin", model.getGround(), SimTK::Vec3(0), 
            SimTK::Vec3(0), *body, SimTK::Vec3(0), SimTK::Vec3(0));
    auto& coord = joint->updCoordinate(PinJoint::Coord::RotationZ);
    coord.setName("rotation");
    model.addComponent(joint);

    auto* actu = new CoordinateActuator("actuator");
    actu->setCoordinate(&coord);
    actu->setOptimalForce(T);
    model.addComponent(actu);

    auto& s = model.initSystem();
    model.printSubcomponentInfo();
    model.printDetailedInfo(s);

    return model;
}

class FinalAngleCost : public MucoCost {
protected:
    void calcEndpointCostImpl(const SimTK::State& finalState,
        SimTK::Real& cost) const override {
        const auto& finalPosition = finalState.getY()[0];

        cost = (finalPosition - FINAL_ANG) * (finalPosition - FINAL_ANG);
    }
};

// Optimize the center of mass for a bar attached a single pin joint ("see-saw") 
// so that the bar remains stationary (i.e. COM located directly above pin 
// joint).
void testRotatingBar() {
    int N = 100;

    MucoTool muco;
    muco.setName("rotating_bar");
    MucoProblem& mp = muco.updProblem();
    mp.setModel(createRotatingBarModel());
    mp.setTimeBounds(0, FINAL_TIME);
    mp.setStateInfo("pin/rotation/value", {0, 2*FINAL_ANG}, 0, 
        {0.75*FINAL_ANG, 1.25*FINAL_ANG});
    mp.setStateInfo("pin/rotation/speed", {0, 2*FINAL_ANGVEL}, 0, FINAL_ANGVEL);
    mp.setControlInfo("actuator", 1.0, 1.0, 1.0);

    // Choose z moment of inertia vector (3rd element of inertia vector).
    MucoParameter inertia("moment_of_inertia_z", "body", "inertia", 
            MucoBounds(0, 10), 2);

    FinalAngleCost cost;
    mp.addCost(cost);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(N);

    MucoSolution sol = muco.solve();
    sol.write("testMucoParameters_testRotatingBar_sol.sto");

    SimTK_TEST_EQ_TOL(sol.getParameter("moment_of_inertia_z"), Izz, 0.005);
}

int main() {
    SimTK_START_TEST("testMucoParameters");
        SimTK_SUBTEST(testOscillatorMass);
        SimTK_SUBTEST(testOneParameterTwoSprings);
        SimTK_SUBTEST(testRotatingBar);
    SimTK_END_TEST();
}