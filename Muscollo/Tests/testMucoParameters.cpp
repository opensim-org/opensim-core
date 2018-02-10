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
OpenSim_DECLARE_CONCRETE_OBJECT(FinalPositionCost, MucoCost);
protected:
    void calcEndpointCostImpl(const SimTK::State& finalState,
            SimTK::Real& cost) const override {
       const auto& finalPosition = finalState.getY()[0];

       cost = (finalPosition - 0.5) * (finalPosition - 0.5);
    }
};

/// Optimize the mass of a simple harmonic oscillator such that it follows the
/// correct trajectory specified by the state bounds and the FinalPositionCost.
/// This tests the ability for MucoParameter to optimize a simple scalar model
/// property value.
void testOscillatorMass() {
    int N = 25;

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

    SimTK_TEST_EQ_TOL(sol.getParameter("oscillator_mass"), MASS, 0.003);
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
    spring1->setStiffness(0.25*STIFFNESS);
    spring1->setViscosity(0.0);
    model.addComponent(spring1);

    auto* spring2 = new SpringGeneralizedForce();
    spring2->setName("spring2");
    spring2->set_coordinate("position");
    spring2->setRestLength(0.0);
    spring2->setStiffness(0.25*STIFFNESS);
    spring2->setViscosity(0.0);
    model.addComponent(spring2);

    return model;
}

/// Optimize the stiffness in two parallel springs and make sure they sum to the 
/// equivalent stiffness of a single spring that would produce the same 
/// oscillation trajectory. This tests the ability for MucoParameter to optimize
/// the value of a model property for two different components.
void testOneParameterTwoSprings() {
    int N = 25;

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
        MucoBounds(0, 100));
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
        0.003);
}

const double L = 1; 
const double xCOM = -0.25*L;
Model createSeeSawModel() {
    Model model;
    model.setName("seesaw");
    model.set_gravity(SimTK::Vec3(0, -9.81, 0));
    // Set body with z-rotational inertia and COM at the geometric center.
    auto* body = new Body("body", MASS, SimTK::Vec3(0),
        SimTK::Inertia(0, 0, 1));
    Ellipsoid bodyGeometry(0.5*L, 0.1*L, 0.1*L); // for visualization
    body->attachGeometry(bodyGeometry.clone());
    model.addComponent(body);

    // Allows rotation around z. Connected offset from the midpoint of the body.
    auto* joint = new PinJoint("pin", model.getGround(), SimTK::Vec3(0, 1, 0), 
            SimTK::Vec3(0), *body, SimTK::Vec3(xCOM, 0, 0), SimTK::Vec3(0));
    auto& coord = joint->updCoordinate(PinJoint::Coord::RotationZ);
    coord.setName("rotation");
    model.addComponent(joint);

    return model;
}

class RotationalAccelerationCost : public MucoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(RotationalAccelerationCost, MucoCost);
protected:
    void calcIntegralCostImpl(const SimTK::State& state,
        SimTK::Real& integrand) const override {

        getModel().realizeAcceleration(state); // TODO would avoid this, ideally.
        const auto& accel = getModel().getStateVariableDerivativeValue(
            state, "pin/rotation/speed");

        integrand = accel * accel;
    }
};

/// Optimize the center of mass location of a body mobilized by pin joint, such
/// that the body comes to rest, i.e. the body's center of mass becomes aligned
/// with the pin joint rotation point. This tests the ability of MucoParameter
/// to optimize an element of a non-scalar model property.
void testSeeSawCOM() {
    int N = 25;

    MucoTool muco;
    muco.setName("seesaw_com");
    MucoProblem& mp = muco.updProblem();
    mp.setModel(createSeeSawModel());
    mp.setTimeBounds(0, 5);
    mp.setStateInfo("pin/rotation/value", {-10, 10}, 0, {-10, 10});
    mp.setStateInfo("pin/rotation/speed", {-10, 10}, 0, {-10, 10});

    // Choose x-location of COM, which is the mass_center property's first 
    // element.
    // The bounds were chosen such that initial guess for x-location of the 
    // body's COM isn't the solution, but close enough for the problem to
    // converge.
    int centerOfMassElt = 0; 
    MucoParameter com("com_location", "body", "mass_center", 
            MucoBounds(-0.7*L, 0), centerOfMassElt);
    mp.addParameter(com);

    RotationalAccelerationCost cost;
    mp.addCost(cost);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(N);

    MucoSolution sol = muco.solve().unseal();
    const auto& sol_xCOM = sol.getParameter("com_location");
    sol.write("testMucoParameters_testSeeSawCOM_sol.sto");
    
    // Update problem model with new mass center.
    // TODO: create method for this, or have MucoTool do it automatically
    // SimTK::Vec3 sol_COM(sol_xCOM, 0, 0);
    // mp.updPhase(0).updModel().updComponent<Body>("body").setMassCenter(sol_COM);

    // Body will be at rest since COM should now be aligned with the pin joint.           
    // muco.visualize(sol);

    SimTK_TEST_EQ_TOL(sol_xCOM, xCOM, 0.003);
}

int main() {
    SimTK_START_TEST("testMucoParameters");
        SimTK_SUBTEST(testOscillatorMass);
        SimTK_SUBTEST(testOneParameterTwoSprings);
        SimTK_SUBTEST(testSeeSawCOM);
    SimTK_END_TEST();
}