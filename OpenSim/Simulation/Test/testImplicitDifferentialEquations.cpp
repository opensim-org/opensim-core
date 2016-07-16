/* -------------------------------------------------------------------------- *
 *              OpenSim:  testImplicitDifferentialEquations.cpp               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2016 Stanford University and the Authors                     *
 * Author(s): Chris Dembia, Brad Humphreys                                    *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/** TODO
*/

// Tests:
// TODO component w/multiple state vars, only some of which have implicit form.
// TODO model containing components with and without explicit form.
// TODO write custom implicit form solver.
// TODO copying and (de)serializing models with implicit forms.
// TODO model containing multiple components with implicit form.
// TODO editing yDotGuess or lambdaGuess invalidates the residual cache.
// TODO calculation of YIndex must be correct.
// TODO test implicit form with multibody system but no auxiliary dynamics, as
//      well as only explicit auxiliary dynamics.
// TODO test given lambda.
// TODO debug by comparing directly to result of calcResidualForce().

// Implementation:
// TODO Only create implicit cache/discrete vars if any components have an
// implicit form (place in extendAddToSystemAfterComponents()).


#include <OpenSim/Simulation/osimSimulation.h>

using namespace OpenSim;
using namespace SimTK;

/** TODO */
class LinearDynamics : public Component {
OpenSim_DECLARE_CONCRETE_OBJECT(LinearDynamics, Component);
public:
    OpenSim_DECLARE_PROPERTY(default_activ, double,
        "Default value of state variable.");
    OpenSim_DECLARE_PROPERTY(coeff, double,
        "Coefficient in the differential equation.");
    OpenSim_DECLARE_OUTPUT_FOR_STATE_VARIABLE(activ);
    LinearDynamics() {
        constructProperty_default_activ(0.0);
        constructProperty_coeff(-1.0);
    }
protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("activ" /* , true has implicit form */);
    }
    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        // TODO invokes false error msg. Super::computeStateVariableDerivatives(s);
        const Real& activ = getStateVariableValue(s, "activ");
        setStateVariableDerivativeValue(s, "activ", get_coeff() * activ);
    }
    void computeImplicitResiduals(const SimTK::State& s)
            const override {
        // TODO Super::computeStateVariableDerivatives(s);
        const Real& activ = getStateVariableValue(s, "activ");
        double activDotGuess = getStateVariableDerivativeGuess(s, "activ");
        setImplicitResidual(s, "activ", get_coeff() * activ - activDotGuess);
    }
    void extendInitStateFromProperties(SimTK::State& s) const override {
        Super::extendInitStateFromProperties(s);
        setStateVariableValue(s, "activ", get_default_activ());
    }
    void extendSetPropertiesFromState(const SimTK::State& s) override {
        Super::extendSetPropertiesFromState(s);
        set_default_activ(getStateVariableValue(s, "activ"));
    }
};

/// Integrates the given system and returns the final state via the `state`
/// argument.
void simulate(const Model& model, State& state, Real finalTime) {
    SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
    SimTK::TimeStepper ts(model.getSystem(), integrator);
    ts.initialize(state);
    ts.stepTo(finalTime);
    state = ts.getState();
}

// Ensure that explicit forward integration works for a component that also
// provides an implicit form. The model contains only one component, which
// contains one state variable.
void testExplicitFormOfImplicitComponent() {
    // Create model.
    Model model; model.setName("model");
    auto* comp = new LinearDynamics();
    comp->setName("foo");
    const Real initialValue = 3.5;
    comp->set_default_activ(initialValue);
    const Real coeff = -0.28;
    comp->set_coeff(coeff);
    model.addComponent(comp);
    
    // TODO auto* rep = new ConsoleReporter();
    // TODO rep->set_report_time_interval(0.01);
    // TODO model.addComponent(rep);
    // TODO rep->updInput("inputs").connect(comp->getOutput("activ"));
    
    auto s = model.initSystem();
    
    // Check initial values.
    SimTK_TEST(comp->getStateVariableValue(s, "activ") == initialValue);
    model.realizeAcceleration(s);
    SimTK_TEST_EQ(comp->getStateVariableDerivativeValue(s, "activ"),
                  coeff * initialValue);
    
    // Simulate and check resulting value of state variable.
    const double finalTime = 0.23;
    simulate(model, s, finalTime);
    SimTK_TEST_EQ_TOL(comp->getStateVariableValue(s, "activ"),
                      initialValue * exp(coeff * finalTime), 1e-5);
}

void testSingleCustomImplicitEquation() {
    Model model; model.setName("model");
    auto* comp = new LinearDynamics();
    comp->setName("foo");
    const Real initialValue = 3.5;
    comp->set_default_activ(initialValue);
    const Real coeff = -0.28;
    comp->set_coeff(coeff);
    model.addComponent(comp);
    auto s = model.initSystem();
    
    // TODO now doing the realize internally to getImplicitResidual();
    // no need to do this check.
    // If not realized to Velocity, can't call getQDot(), which is needed
    // when computing the residuals.
    //SimTK_TEST_MUST_THROW_EXC(comp->getImplicitResidual(s, "activ"),
    //                          SimTK::Exception::StageTooLow);
    //SimTK_TEST_MUST_THROW_EXC(model.getImplicitResiduals(s),
    //                          SimTK::Exception::StageTooLow);
    //
    //// If not realized to Dynamics, get exception for trying to access residual.
    //model.realizeVelocity(s);
    //SimTK_TEST_MUST_THROW_EXC(comp->getImplicitResidual(s, "activ"),
    //                          SimTK::Exception::ErrorCheck);
    //SimTK_TEST_MUST_THROW_EXC(model.getImplicitResiduals(s),
    //                          SimTK::Exception::ErrorCheck);
    
    // Access residual from the component:
    model.realizeDynamics(s); // Must realize to dynamics to get residuals.
    // TODO set yGuess.
    const Real activDotGuess = 5.3;
    comp->setStateVariableDerivativeGuess(s, "activ", activDotGuess);
    double expectedResidual = coeff * initialValue - activDotGuess;
    SimTK_TEST(comp->getImplicitResidual(s, "activ") == expectedResidual);
    
    // Make sure the residual is accessible from the entire residual vector.
    SimTK_TEST_EQ(model.getImplicitResiduals(s), Vector(1, expectedResidual));
    /* TODO
    */
    // TODO set derivative guess in one sweep.
    // TODO model.setYDotGuess(s, Vector(1, activDotGuess));
}

// Test implicit multibody dynamics (i.e., inverse dynamics) with a point
// mass that can move along the direction of gravity.
void testImplicitMultibodyDynamics1DOF() {
    const double g = 9.81;
    const double mass = 7.2;
    const double u_i    = 3.9;
    
    // Build model.
    // ------------
    Model model; model.setName("ball");
    model.setGravity(Vec3(-g, 0, 0)); // gravity pulls in the -x direction.
    auto* body = new OpenSim::Body("ptmass", mass, Vec3(0), Inertia(0));
    auto* slider = new SliderJoint(); slider->setName("slider");
    model.addBody(body);
    model.addJoint(slider);
    slider->updConnector("parent_frame").connect(model.getGround());
    slider->updConnector("child_frame").connect(*body);
    
    State s = model.initSystem();
    const auto& coord = slider->getCoordinateSet()[0];
    coord.setSpeedValue(s, u_i);
    
    const double qDotGuess = 5.6; const double uDotGuess = 8.3;
    
    // Check implicit form.
    // --------------------
    // Set derivative guess.
    coord.setStateVariableDerivativeGuess(s, "value", qDotGuess);
    coord.setStateVariableDerivativeGuess(s, "speed", uDotGuess);
   
    // Get residual.
    Vector expectedResiduals(2);
    expectedResiduals[0] = u_i - qDotGuess;
    expectedResiduals[1] = mass * uDotGuess - mass * (-g); // M u_dot-f_applied
    model.realizeDynamics(s);
    // Check individual elements of the residual.
    SimTK_TEST_EQ(coord.getImplicitResidual(s, "value"), expectedResiduals[0]);
    SimTK_TEST_EQ(coord.getImplicitResidual(s, "speed"), expectedResiduals[1]);
    // Check the entire residuals vector.
    SimTK_TEST_EQ(model.getImplicitResiduals(s), expectedResiduals);
    
    // TODO set YDotGuess all at once.
    
    
    // Check explicit form.
    // --------------------
    model.realizeAcceleration(s);
    Vector expectedYDot(2);
    expectedYDot[0] /* = qdot*/ = u_i; expectedYDot[1] /* = udot*/ = -g;
    SimTK_TEST_EQ(s.getYDot(), expectedYDot);
}

int main() {
    SimTK_START_TEST("testImplicitDifferentialEquatiosns");
        SimTK_SUBTEST(testExplicitFormOfImplicitComponent);
        SimTK_SUBTEST(testSingleCustomImplicitEquation);
        SimTK_SUBTEST(testImplicitMultibodyDynamics1DOF);
    SimTK_END_TEST();
}





