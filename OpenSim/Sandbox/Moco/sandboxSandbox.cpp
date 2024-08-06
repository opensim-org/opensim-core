/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSandbox.cpp                                           *
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

// This file provides a way to easily prototype or test temporary snippets of
// code during development.

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Actuators/SpringGeneralizedForce.h>

using namespace OpenSim;

const double STIFFNESS = 100.0; // N/m
const double MASS = 5.0; // kg
const double FINAL_TIME = SimTK::Pi * sqrt(MASS / STIFFNESS);
std::unique_ptr<Model> createOscillatorModel() {
    auto model = make_unique<Model>();
    model->setName("oscillator");
    model->set_gravity(SimTK::Vec3(0, 0, 0));
    // We will optimize the mass of this body in the test below. Here, we'll set
    // the model with "incorrect" mass value, which the test will use as the
    // initial guess.
    auto* body = new Body("body", 0.5*MASS, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model->getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model->addComponent(joint);

    auto* spring = new SpringGeneralizedForce();
    spring->set_coordinate("position");
    spring->setRestLength(0.0);
    spring->setStiffness(STIFFNESS);
    spring->setViscosity(0.0);
    model->addComponent(spring);

    return model;
}

class FinalPositionGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(FinalPositionGoal, MocoGoal);
protected:
    void initializeOnModelImpl(const Model&) const override {
        setRequirements(0, 1);
    }
    void calcGoalImpl(const GoalInput& input,
            SimTK::Vector& cost) const override {
       const auto& finalPosition = input.final_state.getY()[0];

       cost[0] = (finalPosition - 0.5) * (finalPosition - 0.5);
    }
};

/// Optimize the mass of a simple harmonic oscillator such that it follows the
/// correct trajectory specified by the state bounds and the FinalPositionCost.
/// This tests the ability for MocoParameter to optimize a simple scalar model
/// property value.
void testOscillatorMass() {
    int N = 25;

    MocoStudy study;
    study.setName("oscillator_mass");
    MocoProblem& mp = study.updProblem();
    mp.setModel(createOscillatorModel());
    mp.setTimeBounds(0, FINAL_TIME);
    mp.setStateInfo("/slider/position/value", {-5.0, 5.0}, -0.5, {0.25, 0.75});
    mp.setStateInfo("/slider/position/speed", {-20, 20}, 0, 0);

    mp.addParameter("oscillator_mass", "body", "mass", MocoBounds(0, 10));

    mp.addGoal<FinalPositionGoal>();

    auto& ms = study.initSolver<MocoCasADiSolver>();
    ms.set_num_mesh_intervals(N);

    MocoSolution sol = study.solve();
    //sol.write("testMocoParameters_testOscillatorMass_sol.sto");

    // CHECK(sol.getParameter("oscillator_mass") ==
    //         Catch::Approx(MASS).epsilon(0.003));
}

std::unique_ptr<Model> createOscillatorTwoSpringsModel() {
    auto model = make_unique<Model>();
    model->setName("oscillator_two_springs");
    model->set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", MASS, SimTK::Vec3(0), SimTK::Inertia(0));
    model->addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("slider", model->getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model->addComponent(joint);

    auto* spring1 = new SpringGeneralizedForce();
    spring1->setName("spring1");
    spring1->set_coordinate("position");
    spring1->setRestLength(0.0);
    spring1->setStiffness(0.25*STIFFNESS);
    spring1->setViscosity(0.0);
    model->addComponent(spring1);

    auto* spring2 = new SpringGeneralizedForce();
    spring2->setName("spring2");
    spring2->set_coordinate("position");
    spring2->setRestLength(0.0);
    spring2->setStiffness(0.25*STIFFNESS);
    spring2->setViscosity(0.0);
    model->addComponent(spring2);

    return model;
}

void testSpringParameter() {
    int N = 25;

    MocoStudy study;
    study.setName("oscillator_spring_stiffnesses");
    MocoProblem& mp = study.updProblem();
    mp.setModel(createOscillatorTwoSpringsModel());
    mp.setTimeBounds(0, FINAL_TIME);
    mp.setStateInfo("/slider/position/value", {-5.0, 5.0}, -0.5, {0.25, 0.75});
    mp.setStateInfo("/slider/position/speed", {-20, 20}, 0, 0);

    // Optimize a single stiffness value and apply to both springs.
    std::vector<std::string> components = {"spring1", "spring2"};
    mp.addParameter("spring_stiffness", components, "stiffness",
        MocoBounds(0, 100));

    mp.addGoal<FinalPositionGoal>();

    auto& ms = study.initSolver<MocoCasADiSolver>();
    ms.set_num_mesh_intervals(N);
    ms.set_parameters_require_initsystem(false);

    MocoSolution sol = study.solve();
    //sol.write("testMocoParameters_testOscillatorMassTwoSprings_sol.sto");

    // Since springs add in parallel, both stiffness must be the same value
    // and equal half the original spring stiffness.
    // CHECK(sol.getParameter("spring_stiffness") ==
    //         Catch::Approx(0.5*STIFFNESS).epsilon(0.003));
}

int main() {

    testSpringParameter();

    return EXIT_SUCCESS;
}
