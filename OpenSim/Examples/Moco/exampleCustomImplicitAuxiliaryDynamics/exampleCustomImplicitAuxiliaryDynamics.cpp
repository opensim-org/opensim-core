/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleCustomImplicitAuxiliaryDynamics.cpp                   *
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

#include <OpenSim/Moco/osimMoco.h>

using namespace OpenSim;

/// This class implements a custom component with simple dynamics in implicit
/// and explicit form. The dynamics are defined by the differential
/// equation y y' - 1 = 0. The explicit form is y' = 1 / y.
/// To implement a state variable with implicit auxiliary dynamics, take the
/// following steps:
/// - add a state variable in extendAddToSystem() by invoking
///   addStateVariable();
/// - add an output statebounds_<state-name> to be used by Moco as the default
///   bounds on the state variable (optional);
/// - add an output implicitresidual_<state-name> that provides the value of
///   the implicit differential equation;
/// - add an output implicitenabled_<state-name> that indicates whether implicit
///   dynamics mode is enabled or disabled for this state variable;
/// - add a discrete state variable implicitderiv_<state-name> in
///   extendAddToSystem() to store the derivative of the state variable, which
///   must be used within the implicit differential equation (Moco is
///   responsible for setting the value of this variable);
/// - add a cache variable in extendAddToSystem() to store the value of the
///   implicit residual;
/// - implement the implicit residual output function; and
/// - implement computeStateVariableDerivatives() for both implicit and
///   explicit modes.
class MyImplicitAuxiliaryDynamics : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(MyImplicitAuxiliaryDynamics, Component);

public:
    OpenSim_DECLARE_PROPERTY(
            default_foo, double, "Default value of the state variable foo.");
    OpenSim_DECLARE_PROPERTY(foo_dynamics_mode, std::string,
            "The dynamics mode for enforcing y y' - 1 = 0. "
            "Options: 'explicit' or 'implicit'. Default: 'explicit'. ");

    OpenSim_DECLARE_OUTPUT(
            statebounds_foo, SimTK::Vec2, getBoundsFoo, SimTK::Stage::Model);
    OpenSim_DECLARE_OUTPUT(implicitresidual_foo, double, getImplicitResidualFoo,
            SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(implicitenabled_foo, bool, getImplicitEnabledFoo,
            SimTK::Stage::Model);

    MyImplicitAuxiliaryDynamics() {
        setName("implicit_auxdyn");
        constructProperty_default_foo(0.5);
        constructProperty_foo_dynamics_mode("explicit");
    }
    MyImplicitAuxiliaryDynamics(std::string foo_dynamics_mode)
            : MyImplicitAuxiliaryDynamics() {
        set_foo_dynamics_mode(foo_dynamics_mode);
    }

    SimTK::Vec2 getBoundsFoo(const SimTK::State&) const {
        return SimTK::Vec2(0, 3);
    }
    // This function requires a state parameter because we use it in an Output;
    // however, we don't actually need the state.
    bool getImplicitEnabledFoo(const SimTK::State&) const { return true; }
    // We use "residual" to refer to the value of the implicit differential
    // equation function f(y, y') = y y' - 1.
    double getImplicitResidualFoo(const SimTK::State& s) const {
        // If the dynamics mode is not implicit, then this function should not
        // be invoked, so we return NaN.
        if (!m_foo_dynamics_mode_implicit) { return SimTK::NaN; }
        if (!isCacheVariableValid(s, "implicitresidual_foo")) {
            // Get the derivative control value.
            const double derivative =
                    getDiscreteVariableValue(s, "implicitderiv_foo");
            // Get the state variable value.
            const double statevar = getStateVariableValue(s, "foo");
            // The dynamics residual: y y' - 1 = 0.
            double residual = derivative * statevar - 1;
            // Update the cache variable with the residual value.
            setCacheVariableValue(s, "implicitresidual_foo", residual);
            markCacheVariableValid(s, "implicitresidual_foo");
        }
        return getCacheVariableValue<double>(s, "implicitresidual_foo");
    }

private:
    void extendFinalizeFromProperties() override {
        OPENSIM_THROW_IF_FRMOBJ(get_foo_dynamics_mode() != "implicit" &&
                                        get_foo_dynamics_mode() != "explicit",
                Exception,
                "Expected foo_dynamics_mode to be 'implicit' or "
                "'explicit', but got '{}'.",
                get_foo_dynamics_mode());
        m_foo_dynamics_mode_implicit = get_foo_dynamics_mode() == "implicit";
    }
    void extendInitStateFromProperties(SimTK::State& s) const override {
        Super::extendInitStateFromProperties(s);
        setStateVariableValue(s, "foo", get_default_foo());
    }
    void extendSetPropertiesFromState(const SimTK::State& s) override {
        Super::extendSetPropertiesFromState(s);
        set_default_foo(getStateVariableValue(s, "foo"));
    }
    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        if (m_foo_dynamics_mode_implicit) {
            // In implicit mode, the state variable derivative is the value
            // of the discrete variable. We must set this for Moco to solve the
            // problem correctly.
            const double derivative =
                    getDiscreteVariableValue(s, "implicitderiv_foo");
            setStateVariableDerivativeValue(s, "foo", derivative);
        } else {
            const double statevar = getStateVariableValue(s, "foo");
            setStateVariableDerivativeValue(s, "foo", 1.0 / statevar);
        }
    }
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("foo");
        addDiscreteVariable("implicitderiv_foo", SimTK::Stage::Dynamics);
        addCacheVariable(
                "implicitresidual_foo", double(0), SimTK::Stage::Dynamics);
    }
    bool m_foo_dynamics_mode_implicit;
};

int main() {
    try {
        double finalFooTimeStepping;
        double finalFooDircol;
        // Simulate the differential equation in explicit mode using one of
        // OpenSim's time-stepping integrators.
        {
            Model model;
            model.addComponent(new MyImplicitAuxiliaryDynamics("explicit"));
            SimTK::State state = model.initSystem();
            model.setStateVariableValue(state, "/implicit_auxdyn/foo", 1.0);
            Manager manager(model, state);
            state = manager.integrate(1.0);
            finalFooTimeStepping =
                    model.getStateVariableValue(state, "/implicit_auxdyn/foo");
        }
        // Simulate the differential equation in implicit mode using Moco.
        // Because this problem does not have any cost terms or control
        // variables, Moco is solving an initial value problem (integrating
        // forward in time from a given initial state), not an optimal control
        // problem.
        {
            MocoStudy study;
            auto& problem = study.updProblem();
            auto model = OpenSim::make_unique<Model>();
            model->addComponent(new MyImplicitAuxiliaryDynamics("implicit"));
            problem.setModel(std::move(model));
            problem.setTimeBounds(0, 1.0);
            problem.setStateInfo("/implicit_auxdyn/foo", {0, 3}, 1.0);
            MocoSolution solution = study.solve();
            const int N = solution.getNumTimes();
            finalFooDircol = solution.getStatesTrajectory().getElt(N - 1, 0);
        }
        std::cout << "Final foo with time-stepping: " << finalFooTimeStepping
                  << "\nFinal foo with Moco: " << finalFooDircol << std::endl;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (...) { return EXIT_FAILURE; }
    return EXIT_SUCCESS;
}
