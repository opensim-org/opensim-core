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
// TODO ensure residuals are NaN if lambdas are not set to 0, unless the system
//      does not have any constraints.
// TODO test calcImplicitResidualsAndConstraintErrors(); make sure constraintErrs
//      is empty if there are no constraints. Make sure lambdaGuess is the
//      correct size.
// TODO test prescribedmotion, model with non-implicit dynamics.
// TODO put multibody residuals calculation in extendComputeImplicitResiduals().
// TODO test what happens when you provide an implicit from but don't set hasImplicitForm.
// TODO test what happens when you set hasImplicitForm() but don't provide it.
// TODO test inheritance hierarchy: multiple subclasses add state vars.

// TODO mock up the situation where we are minimizing joint contact load
//      while obeying dynamics (in direct collocation fashion--the entire
//      trajectory).

// Implementation:
// TODO Only create implicit cache/discrete vars if any components have an
// implicit form (place in extendAddToSystemAfterComponents()).
// TODO if we use the operator form (not storing residual etc in the state),
// then we have to be clear to implementors of the residual equation that they
// CANNOT depend on qdot, udot, zdot or ydot vectors in the state; that will
// invoke forward dynamics!
// TODO handle quaternion constraints.

// TODO sketch out solver-like interface (using IPOPT).


#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Simulation/osimSimulation.h>

using namespace OpenSim;
using namespace SimTK;

/** TODO change this to have a different explicit vs implicit form.
TODO otherwise we can't tell if we are using the default implicit form or
the provided one. */
class LinearDynamicsExplicitImplicit : public Component {
OpenSim_DECLARE_CONCRETE_OBJECT(LinearDynamicsExplicitImplicit, Component);
public:
    OpenSim_DECLARE_PROPERTY(default_activ, double,
        "Default value of state variable.");
    OpenSim_DECLARE_PROPERTY(coeff, double,
        "Coefficient in the differential equation.");
    LinearDynamicsExplicitImplicit() {
        constructProperty_default_activ(0.0);
        constructProperty_coeff(-1.0);
    }
protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("activ", SimTK::Stage::Dynamics, true
         /* TODO , true has implicit form */);
    }
    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        // TODO invokes false error msg. Super::computeStateVariableDerivatives(s);
        const Real& activ = getStateVariableValue(s, "activ");
        setStateVariableDerivativeValue(s, "activ", get_coeff() * activ);
    }
    void extendComputeImplicitResiduals(const SimTK::State& s) const override {
        Super::extendComputeImplicitResiduals(s);
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

// TODO
class QuadraticDynamicsExplicitOnly : public Component {
OpenSim_DECLARE_CONCRETE_OBJECT(QuadraticDynamicsExplicitOnly, Component);
public:
    OpenSim_DECLARE_PROPERTY(default_length, double,
        "Default value of state variable.");
    OpenSim_DECLARE_PROPERTY(coeff, double,
        "Coefficient in the differential equation.");
    QuadraticDynamicsExplicitOnly() {
        constructProperty_default_length(0.0);
        constructProperty_coeff(-1.0);
    }
protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("length", SimTK::Stage::Dynamics, false);
    }
    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        const Real& activ = getStateVariableValue(s, "length");
        setStateVariableDerivativeValue(s, "length", get_coeff() * activ * activ);
    }
    void extendInitStateFromProperties(SimTK::State& s) const override {
        Super::extendInitStateFromProperties(s);
        setStateVariableValue(s, "length", get_default_length());
    }
    void extendSetPropertiesFromState(const SimTK::State& s) override {
        Super::extendSetPropertiesFromState(s);
        set_default_length(getStateVariableValue(s, "length"));
    }
};

/** // TODO
class MixedExplicitImplicitDynamics : public Component {
OpenSim_DECLARE_CONCRETE_OBJECT(MixedExplicitImplicitDynamics, Component);
public:
    OpenSim_DECLARE_PROPERTY(default_length, double,
        "Default value of state variable.");
    OpenSim_DECLARE_PROPERTY(coeff, double,
        "Coefficient in the differential equation.");
    MixedExplicitImplicitDynamics() {
        constructProperty_default_length(0.0);
        constructProperty_coeff(-1.0);
    }
protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("length");
    }
    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        const Real& activ = getStateVariableValue(s, "length");
        setStateVariableDerivativeValue(s, "length", get_coeff() * activ * activ);
    }
    void extendInitStateFromProperties(SimTK::State& s) const override {
        Super::extendInitStateFromProperties(s);
        setStateVariableValue(s, "length", get_default_length());
    }
    void extendSetPropertiesFromState(const SimTK::State& s) override {
        Super::extendSetPropertiesFromState(s);
        set_default_length(getStateVariableValue(s, "length"));
    }
};
*/

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
    auto* comp = new LinearDynamicsExplicitImplicit();
    comp->setName("foo");
    const Real initialValue = 3.5;
    comp->set_default_activ(initialValue);
    const Real coeff = -0.28;
    comp->set_coeff(coeff);
    model.addComponent(comp);
    
    auto s = model.initSystem();
    
    // Check boolean indicators.
    SimTK_TEST(comp->hasFullImplicitFormThisComponent());
    SimTK_TEST(comp->hasFullImplicitForm());
    SimTK_TEST(model.hasFullImplicitFormThisComponent());
    SimTK_TEST(model.hasFullImplicitForm());
    
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
        auto* comp = new LinearDynamicsExplicitImplicit();
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
    Model model; model.setName("model");
    model.setGravity(Vec3(-g, 0, 0)); // gravity pulls in the -x direction.
    auto* body = new OpenSim::Body("ptmass", mass, Vec3(0), Inertia(0));
    auto* slider = new SliderJoint(); slider->setName("slider");
    model.addBody(body);
    model.addJoint(slider);
    slider->updConnector("parent_frame").connect(model.getGround());
    slider->updConnector("child_frame").connect(*body);
    
    State s = model.initSystem();
    
    const auto& coord = slider->getCoordinateSet()[0];
    
    SimTK_TEST(coord.hasFullImplicitFormThisComponent());
    SimTK_TEST(model.hasFullImplicitForm());
    
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

// When components provide dynamics in explicit form only, we compute the
// implicit form by using the explicit form.
void testImplicitFormOfExplicitOnlyComponent() {
    auto createModel = [](double initialValue, double coeff) -> Model {
        Model model; model.setName("model");
        auto* comp = new QuadraticDynamicsExplicitOnly();
        comp->setName("foo");
        comp->set_default_length(initialValue);
        comp->set_coeff(coeff);
        model.addComponent(comp);
        return model;
    };
    
    const Real initialValue = 1.8;
    const Real coeff = -0.73;
    const Real lengthDotGuess = -4.5;
    // The default implicit residual.
    const Real expectedResidual = coeff * pow(initialValue, 2) - lengthDotGuess;
    
    { // Setting elements of guess by name.
        // TODO
        Model model = createModel(initialValue, coeff);
        State s = model.initSystem();
        const auto& comp = model.getComponent("foo");
        
        // Check boolean indicators.
        SimTK_TEST(!comp.hasFullImplicitFormThisComponent());
        SimTK_TEST(!comp.hasFullImplicitForm());
        SimTK_TEST(model.hasFullImplicitFormThisComponent());
        SimTK_TEST(!model.hasFullImplicitForm());
        
        // Set guess.
        comp.setStateVariableDerivativeGuess(s, "length", lengthDotGuess);
        
        // Access residual by name from the component.
        SimTK_TEST(comp.getImplicitResidual(s, "length") == expectedResidual);
        
        // Access residual from the entire residual vector.
        SimTK_TEST_EQ(model.getImplicitResiduals(s), Vector(1, expectedResidual));
    }
    
    { // Setting entire guess vector at once.
        Model model = createModel(initialValue, coeff);
        State s = model.initSystem();
        const auto& comp = model.getComponent("foo");
        
        // Set guess.
        model.setYDotGuess(s, Vector(1, lengthDotGuess));
        
        // Access residual by name from the component.
        SimTK_TEST(comp.getImplicitResidual(s, "length") == expectedResidual);
        
        // Access residual from the entire residual vector.
        SimTK_TEST_EQ(model.getImplicitResiduals(s), Vector(1, expectedResidual));
        
        // Editing guesses causes residual to be recalculated.
        model.setYDotGuess(s, Vector(1, NaN));
        SimTK_TEST_NOTEQ(model.getImplicitResiduals(s),
                         Vector(1, expectedResidual));
    }
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
    // Solve for the derivatives of the system, ydot, and the Lagrange
    // multipliers, lambda.
    void solve(const State& s, Vector& yDot, Vector& lambda);
private:
    Model m_model;
    std::unique_ptr<Problem> m_problem;
    std::unique_ptr<Optimizer> m_opt;
};
class ImplicitSystemDerivativeSolver::Problem : public SimTK::OptimizerSystem {
public:
    Problem(const ImplicitSystemDerivativeSolver& parent): m_parent(parent) {}
    void setWorkingState(const State& s) { m_workingState = s; }
    int objectiveFunc(const Vector& guess, bool newParams,
                      Real& f) const override {
        f = 0; return 0;
    }
    int constraintFunc(const Vector& guess, bool newParams,
                       Vector& constraints) const override {
        const int ny = m_workingState.getNY();
        Vector yDotGuess; yDotGuess.viewAssign(guess(0, ny));
        Vector lambdaGuess; lambdaGuess.viewAssign(guess(ny, guess.size() - ny));
        
        Vector residuals; residuals.viewAssign(constraints(0, ny));
        Vector pvaerrs; pvaerrs.viewAssign(constraints(ny, guess.size() - ny));
        
        m_parent.m_model.calcImplicitResidualsAndConstraintErrors(m_workingState,
                    yDotGuess, lambdaGuess, // inputs
                    residuals, pvaerrs);    // outputs
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
    const int N = state.getNY() + state.getNMultipliers();
    m_problem->setNumParameters(N);
    m_problem->setNumEqualityConstraints(N);
    Vector limits(N, 50.0); // TODO arbitrary.
    m_problem->setParameterLimits(-limits, limits);
    
    // Set up Optimizer.
    m_opt.reset(new Optimizer(*m_problem, SimTK::InteriorPoint));
    m_opt->useNumericalGradient(true); m_opt->useNumericalJacobian(true);
}
void ImplicitSystemDerivativeSolver::solve(const State& s,
                                           Vector& yDot, Vector& lambda) {
    m_problem->setWorkingState(s);
    Vector results(m_problem->getNumParameters(), 0.0); // TODO inefficient
    m_opt->optimize(results);
    m_opt->setDiagnosticsLevel(1);
    yDot = results(0, s.getNY());
    lambda = results(s.getNY(), results.size() - s.getNY());
}
// end ImplicitSystemDerivativeSolver...........................................

// TODO explain purpose of this test.
void testGenericInterfaceForImplicitSolver() /*TODO rename test */ {
    Model model;
    auto* comp = new LinearDynamicsExplicitImplicit();
    comp->setName("foo");
    const Real initialValue = 3.5;
    comp->set_default_activ(initialValue);
    const Real coeff = -0.21;
    comp->set_coeff(coeff);
    model.addComponent(comp);
    State s = model.initSystem();
    
    // Computing yDot using implicit form.
    Vector yDotImplicit;
    {
        State sImplicit = s;
        ImplicitSystemDerivativeSolver solver(model);
        Vector lambda;
        solver.solve(sImplicit, yDotImplicit, lambda);
        // There should be no multipliers.
        SimTK_TEST(lambda.size() == 0);
    }
    
    // Computing yDot using explicit form.
    Vector yDotExplicit;
    {
        State sExplicit = s;
        model.realizeAcceleration(sExplicit);
        yDotExplicit = sExplicit.getYDot();
    }
    
    // Implicit and explicit forms give same yDot.
    SimTK_TEST_EQ_TOL(yDotImplicit, yDotExplicit, 1e-11);
    // Also make sure the test is actually testing something.
    SimTK_TEST(yDotExplicit.size() == 1);
    SimTK_TEST_NOTEQ(yDotExplicit.norm(), 0);
}

void testImplicitSystemDerivativeSolverMultibody1DOF() {
    Model model; model.setName("model");
    model.setGravity(Vec3(-9.81, 0, 0)); // gravity pulls in the -x direction.
    auto* body = new OpenSim::Body("ptmass", 1.3, Vec3(0), Inertia(0));
    auto* slider = new SliderJoint(); slider->setName("slider");
    model.addBody(body);
    model.addJoint(slider);
    slider->updConnector("parent_frame").connect(model.getGround());
    slider->updConnector("child_frame").connect(*body);
    
    State s = model.initSystem();
    const auto& coord = slider->getCoordinateSet()[0];
    coord.setSpeedValue(s, 1.7);
    
    // Computing yDot using implicit form.
    Vector yDotImplicit;
    {
        State sImplicit = s;
        ImplicitSystemDerivativeSolver solver(model);
        Vector lambda; // unused.
        solver.solve(sImplicit, yDotImplicit, lambda);
    }
    
    // Computing yDot using explicit form.
    Vector yDotExplicit;
    {
        State sExplicit = s;
        model.realizeAcceleration(sExplicit);
        yDotExplicit = sExplicit.getYDot();
    }
    
    // Implicit and explicit forms give same yDot.
    SimTK_TEST_EQ_TOL(yDotImplicit, yDotExplicit, 1e-8);
    
    // Also make sure the test is actually testing something.
    SimTK_TEST(yDotExplicit.size() == 2);
    SimTK_TEST_NOTEQ(yDotExplicit.norm(), 0);
}

void testCoordinateCouplerConstraint() {
    Model model; model.setName("twodof");
    auto* body = new OpenSim::Body("ptmass", 1.5, Vec3(0.7, 0, 0),
                                   Inertia::ellipsoid(1, 2, 3));
    // TODO BallJoint() causes issues.
    auto* joint = new GimbalJoint(); joint->setName("joint");
    auto& c0 = joint->upd_CoordinateSet()[0]; c0.setName("c0");
    auto& c1 = joint->upd_CoordinateSet()[1]; c1.setName("c1");
    auto& c2 = joint->upd_CoordinateSet()[2]; c2.setName("c2");
    model.addBody(body);
    model.addJoint(joint);
    joint->updConnector("parent_frame").connect(model.getGround());
    joint->updConnector("child_frame").connect(*body);
    
    // Set up constraint using a linear relation between c0 and c1.
    auto* coupler = new CoordinateCouplerConstraint(); coupler->setName("cplr");
    model.addConstraint(coupler);
    Array<std::string> indepCoords; indepCoords.append("c0");
    coupler->setIndependentCoordinateNames(indepCoords);
    coupler->setDependentCoordinateName("c1");
    const Real slope = 5.1; const Real intercept = 2.31;
    coupler->setFunction(LinearFunction(slope, intercept));
    
    State s = model.initSystem();
    c0.setValue(s, 0.51, false); // We'll enforce constraints all at once.
    c2.setValue(s, 0.36, false);
    c0.setSpeedValue(s, 1.5); // The projection will change this value.
    c2.setSpeedValue(s, 2.8);
    model.assemble(s);
    model.getSystem().projectU(s); // Enforce velocity constraints.
    
    // Ensure that the constraints are obeyed in the current state.
    SimTK_TEST_EQ(c1.getValue(s), slope * c0.getValue(s) + intercept);
    // Check the velocity-level constraint:
    SimTK_TEST_EQ(c1.getSpeedValue(s), slope * c0.getSpeedValue(s));
    
    // Computing yDot and lambda using implicit form.
    Vector yDotImplicit, lambdaImplicit;
    {
        State sImplicit = s;
        ImplicitSystemDerivativeSolver solver(model);
        solver.solve(sImplicit, yDotImplicit, lambdaImplicit);
        
        // Acceleration-level constraint equation is satisfied.
        SimTK_TEST_EQ_TOL(yDotImplicit[1], slope * yDotImplicit[0], 1e-12);
    }
    
    // Computing yDot and lambda using explicit form.
    Vector yDotExplicit, lambdaExplicit;
    {
        State sExplicit = s;
        model.realizeAcceleration(sExplicit);
        yDotExplicit = sExplicit.getYDot();
        lambdaExplicit = sExplicit.getMultipliers();
        
        // Acceleration-level constraint equation is satisfied.
        SimTK_TEST_EQ_TOL(c1.getAccelerationValue(sExplicit),
                          slope * c0.getAccelerationValue(sExplicit), 1e-9);
    }
    
    
    // Implicit and explicit forms give same yDot and lambda.
    SimTK_TEST_EQ_TOL(yDotImplicit, yDotExplicit, 1e-9);
    SimTK_TEST_EQ_TOL(lambdaImplicit, lambdaExplicit, 1e-9);
    
    // Also make sure the test is actually testing something.
    SimTK_TEST(yDotExplicit.size() == 6); // 3 DOFs, 2 states per DOF.
    SimTK_TEST(lambdaExplicit.size() == 1);
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
        SimTK_SUBTEST(testImplicitFormOfExplicitOnlyComponent);
        // TODO SimTK_SUBTEST(testImplicitFormOfCombinedImplicitAndExplicitComponents);
        // TODO SimTK_SUBTEST(testMultibody1DOFAndCustomComponent);
        SimTK_SUBTEST(testGenericInterfaceForImplicitSolver);
        SimTK_SUBTEST(testImplicitSystemDerivativeSolverMultibody1DOF);
        SimTK_SUBTEST(testCoordinateCouplerConstraint);
        SimTK_SUBTEST(testErrorsForUnsupportedModels);
    SimTK_END_TEST();
}





