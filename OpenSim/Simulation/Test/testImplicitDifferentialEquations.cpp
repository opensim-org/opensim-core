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
// TODO test coupled system (auxiliary dynamics depends on q, u depends on
//      auxiliary dynamics).
// TODO trying to set a yDotGuess or Lambda that is of the wrong size.
// smss.getRep().calcConstraintAccelerationErrors

// Implementation:
// TODO Only create implicit cache/discrete vars if any components have an
// implicit form (place in extendAddToSystemAfterComponents()).
// TODO if we use the operator form (not storing residual etc in the state),
// then we have to be clear to implementors of the residual equation that they
// CANNOT depend on qdot, udot, zdot or ydot vectors in the state; that will
// invoke forward dynamics!

// TODO sketch out solver-like interface (using IPOPT).


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
    auto createModel = [](double initialValue, double coeff) -> Model {
        Model model; model.setName("model");
        auto* comp = new LinearDynamics();
        comp->setName("foo");
        comp->set_default_activ(initialValue);
        comp->set_coeff(coeff);
        model.addComponent(comp);
        return model;
    };
    
    const Real initialValue = 3.5;
    const Real coeff = -0.28;
    const Real activDotGuess = 5.3;
    const Real expectedResidual = coeff * initialValue - activDotGuess;
    
    { // Setting elements of guess by name.
        Model model = createModel(initialValue, coeff);
        State s = model.initSystem();
        const auto& comp = model.getComponent("foo");
        
        // Set guess.
        comp.setStateVariableDerivativeGuess(s, "activ", activDotGuess);
        
        // Access residual by name from the component:
        SimTK_TEST(comp.getImplicitResidual(s, "activ") == expectedResidual);
        
        // Access residual from the entire residual vector.
        SimTK_TEST_EQ(model.getImplicitResiduals(s), Vector(1, expectedResidual));
    }
    
    { // Setting entire guess vector at once.
        Model model = createModel(initialValue, coeff);
        State s = model.initSystem();
        const auto& comp = model.getComponent("foo");
        
        // Set guess.
        model.setYDotGuess(s, Vector(1, activDotGuess));
        
        // Access residual by name from the component.
        SimTK_TEST(comp.getImplicitResidual(s, "activ") == expectedResidual);
        
        // Access residual from the entire residual vector.
        SimTK_TEST_EQ(model.getImplicitResiduals(s), Vector(1, expectedResidual));
        
        // Editing guesses causes residual to be recalculated.
        model.setYDotGuess(s, Vector(1, NaN)); // different guess.
        SimTK_TEST_NOTEQ(model.getImplicitResiduals(s),
                         Vector(1, expectedResidual));
    }
}

// Test implicit multibody dynamics (i.e., inverse dynamics) with a point
// mass that can move along the direction of gravity.
void testImplicitMultibodyDynamics1DOF() {
    const double g = 9.81;
    const double mass = 7.2;
    const double u_i  = 3.9;
    
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
    expectedYDot[0] = u_i /* = qdot*/; expectedYDot[1] = -g /* = udot*/;
    SimTK_TEST_EQ(s.getYDot(), expectedYDot);
    
    // Error-checking.
    // ---------------
    // Size of yDotGuess must be correct.
    SimTK_TEST_MUST_THROW_EXC(model.setYDotGuess(s, Vector(1, 0.)), // too small.
                              SimTK::Exception::Assert);
    SimTK_TEST_MUST_THROW_EXC(model.setYDotGuess(s, Vector(3, 0.)), // too large.
                              SimTK::Exception::Assert);
    
    // Size of residuals must be correct.
    SimTK_TEST_MUST_THROW_EXC(model.updImplicitResiduals(s) = Vector(3, 0.),
                              SimTK::Exception::Cant);
    
    // TODO test size of lambdaGuess.
}

// =============================================================================
// ImplicitSystemDerivativeSolver
// =============================================================================
// TODO clarify: just solving system of nonlinear equations; no cost function.
// Given a state y and constraints f(y, ydot) = 0, solve for ydot.
class ImplicitSystemDerivativeSolver {
public:
    class Problem; // Defined below.
    ImplicitSystemDerivativeSolver(const Model& model);
    // Solve for the derivatives of the system, ydot.
    Vector solve(const State& s);
private:
    Model m_model;
    std::unique_ptr<Problem> m_problem;
    std::unique_ptr<Optimizer> m_opt;
};
class ImplicitSystemDerivativeSolver::Problem : public SimTK::OptimizerSystem {
public:
    Problem(const ImplicitSystemDerivativeSolver& parent): m_parent(parent) {}
    void setWorkingState(const State& s) { m_workingState = s; }
    int objectiveFunc(const Vector& yDotGuess, bool newParams,
                      Real& f) const override {
        f = 0; return 0;
    }
    int constraintFunc(const Vector& yDotGuess, bool newParams,
                       Vector& constraints) const override {
        m_parent.m_model.setYDotGuess(m_workingState, yDotGuess);
        constraints = m_parent.m_model.getImplicitResiduals(m_workingState);
        // TODO lambdas will be NaN? residuals will be bad
        // what to do with lambdas for a system without constraints?
        return 0;
    }
private:
    const ImplicitSystemDerivativeSolver& m_parent;
    // Mutable since we edit the yDotGuess discrete state var in constraintFunc.
    mutable State m_workingState;
};
ImplicitSystemDerivativeSolver::ImplicitSystemDerivativeSolver(
        const Model& model) : m_model(model), m_problem(new Problem(*this)) {
    // Set up Problem.
    State state = m_model.initSystem();
    m_problem->setNumParameters(state.getNY());
    m_problem->setNumEqualityConstraints(state.getNY());
    Vector limits(state.getNY(), 10.0); // TODO arbitrary.
    m_problem->setParameterLimits(-limits, limits);
    
    // Set up Optimizer.
    m_opt.reset(new Optimizer(*m_problem));
    m_opt->useNumericalGradient(true); m_opt->useNumericalJacobian(true);
}
Vector ImplicitSystemDerivativeSolver::solve(const State& s) {
    m_problem->setWorkingState(s);
    Vector results(m_problem->getNumParameters(), 0.0); // TODO inefficient
    m_opt->optimize(results);
    m_opt->setDiagnosticsLevel(1);
    return results;
}
// end ImplicitSystemDerivativeSolver...........................................

void testGenericInterfaceForImplicitSolver() /*TODO rename test */ {
    Model model;
    auto* comp = new LinearDynamics();
    comp->setName("foo");
    const Real initialValue = 3.5;
    comp->set_default_activ(initialValue);
    const Real coeff = -0.28;
    comp->set_coeff(coeff);
    model.addComponent(comp);
    State s = model.initSystem();
    
    // Computing yDot using implicit form.
    ImplicitSystemDerivativeSolver solver(model);
    Vector yDotImplicit = solver.solve(s);
    
    // Computing yDot using explicit form.
    model.realizeAcceleration(s);
    const Vector& yDotExplicit = s.getYDot();
    
    // Implicit and explicit forms give same yDot.
    SimTK_TEST_EQ_TOL(yDotImplicit, yDotExplicit, 1e-11);
}

void testErrorsForUnsupportedModels() {
    // TODO constraints? does not require error message.
    // TODO prescribed motion.
}

int main() {
    SimTK_START_TEST("testImplicitDifferentialEquations");
        SimTK_SUBTEST(testExplicitFormOfImplicitComponent);
        SimTK_SUBTEST(testSingleCustomImplicitEquation);
        SimTK_SUBTEST(testImplicitMultibodyDynamics1DOF);
        // TODO SimTK_SUBTEST(testMultibody1DOFAndCustomComponent);
        SimTK_SUBTEST(testGenericInterfaceForImplicitSolver);
        SimTK_SUBTEST(testErrorsForUnsupportedModels);
    SimTK_END_TEST();
}





