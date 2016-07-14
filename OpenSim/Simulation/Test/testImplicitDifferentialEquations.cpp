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

// TODO component w/multiple state vars, only some of which have implicit form.
// TODO model containing components with and without explicit form.
// TODO write custom implicit form solver.


#include <OpenSim/Simulation/osimSimulation.h>

using namespace OpenSim;
using namespace SimTK;

/** TODO */
class TrigDynamics : public Component {
OpenSim_DECLARE_CONCRETE_OBJECT(TrigDynamics, Component);
public:
    OpenSim_DECLARE_PROPERTY(default_sine, double,
        "Default value of state variable.");
    OpenSim_DECLARE_OUTPUT_FOR_STATE_VARIABLE(sine);
    TrigDynamics() {
        constructProperty_default_sine(0);
    }
protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("sine" /* , true has implicit form */);
    }
    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        // TODO Super::computeStateVariableDerivatives(s);
        setStateVariableDerivativeValue(s, "sine", cos(s.getTime()));
    }
    /* TODO
    void computeStateVariableImplicitResiduals(const SimTK::State& s)
            const override {
        Super::computeStateVariableDerivatives(s);
        yDotGuess = getStateVariableDerivativeGuess(s, "sine");
        setStateVariableImplicitResidual(s, "sine", cos(s.getTime()) - yDotGuess);
    }
    */
    void extendInitStateFromProperties(SimTK::State& s) const override {
        Super::extendInitStateFromProperties(s);
        setStateVariableValue(s, "sine", get_default_sine());
    }
    void extendSetPropertiesFromState(const SimTK::State& s) override {
        Super::extendSetPropertiesFromState(s);
        set_default_sine(getStateVariableValue(s, "sine"));
    }
};

/// Integrates the given system and returns the final state via the `state`
/// argument.
void simulate(const Model& model, State& state, Real finalTime) {
    SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
    SimTK::TimeStepper ts(model.getSystem(), integrator);
    ts.initialize(state);
    // TODO ts.setReportAllSignificantStates(true);
    // TODO integrator.setReturnEveryInternalStep(true);
    ts.stepTo(finalTime);
    state = ts.getState();
}

// Ensure that explicit forward integration works for a component that also
// provides an implicit form.
void testExplicitFormOfImplicitComponent() {
    Model model; model.setName("model");
    auto* trig = new TrigDynamics();
    trig->setName("foo");
    const Real initialValue = 3.5;
    trig->set_default_sine(initialValue);
    model.addComponent(trig);
    
    // TODO auto* rep = new ConsoleReporter();
    // TODO rep->set_report_time_interval(0.01);
    // TODO model.addComponent(rep);
    // TODO rep->updInput("inputs").connect(trig->getOutput("sine"));
    
    auto s = model.initSystem();
    SimTK_TEST(trig->getStateVariableValue(s, "sine") == initialValue);
    model.realizeAcceleration(s);
    SimTK_TEST(trig->getStateVariableDerivativeValue(s, "sine") == cos(s.getTime()));
    
    const double finalTime = 0.23;
    simulate(model, s, finalTime);
    SimTK_TEST_EQ_TOL(trig->getStateVariableValue(s, "sine"),
                      initialValue + sin(finalTime), 1e-5);
}

int main() {
    SimTK_START_TEST("testImplicitDifferentialEquatiosns");
        SimTK_SUBTEST(testExplicitFormOfImplicitComponent);
    SimTK_END_TEST();
}