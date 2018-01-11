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
#include <OpenSim/Actuators/SpringGeneralizedForce.h>


using namespace OpenSim;

const double PI = 3.14159265358979323846;
const double STIFFNESS = 100.0; // N/m
const double MASS = 5.0; // kg
const double FINAL_TIME = PI * sqrt(MASS / STIFFNESS);

Model createOscillatorModel() {
    Model model;
    model.setName("oscillator");
    model.set_gravity(SimTK::Vec3(0, 0, 0));
    // Set body with incorrect mass value.
    auto* body = new Body("body", 7.5, SimTK::Vec3(0), SimTK::Inertia(0));
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

    auto& s = model.initSystem();
    model.printDetailedInfo(s);
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
    int N = 50;

    MucoTool muco;
    muco.setName("oscillator_mass");
    MucoProblem& mp = muco.updProblem();
    mp.setModel(createOscillatorModel());
    mp.setTimeBounds(0, FINAL_TIME);
    mp.setStateInfo("slider/position/value", {-5.0, 5.0}, -0.5, {0.25, 0.75});
    mp.setStateInfo("slider/position/speed", {-20, 20}, 0, 0);
    
    MucoParameter mass("oscillator_mass", "mass", "body", MucoBounds(0, 10));
    mp.addParameter(mass);

    FinalPositionCost cost;
    mp.addCost(cost);

    MucoTropterSolver& ms = muco.initSolver();
    ms.set_num_mesh_points(N);

    MucoSolution sol = muco.solve();
    sol.write("testMucoParameters_testOscillatorMass_sol.sto");

    
    std::cout << "debug: " << sol.getParameter("oscillator_mass") << std::endl;

    SimTK_TEST_EQ_TOL(sol.getParameter("oscillator_mass"), MASS, 0.005);

}


int main() {
    SimTK_START_TEST("testMucoParameters");
        SimTK_SUBTEST(testOscillatorMass);
    SimTK_END_TEST();
}