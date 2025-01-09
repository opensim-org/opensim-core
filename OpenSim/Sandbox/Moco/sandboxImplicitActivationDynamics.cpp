/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxImplicitActivationDynamics.cpp                        *
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
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <tropter/tropter.h>
#include <Moco/InverseMuscleSolver/INDYGO.h>
#include <Moco/InverseMuscleSolver/DeGrooteFregly2016MuscleStandalone.h>

using namespace OpenSim;
using tropter::VectorX;


class SimpleInverseTendonForceState
        : public tropter::Problem<adouble> {
public:
    using T = adouble;
    const double max_isometric_force = 30;
    const double optimal_fiber_length = 0.10;
    const double tendon_slack_length = 0.05;
    const double pennation_angle_at_optimal = 0.1;
    // optimal fiber lengths per second:
    const double max_contraction_velocity = 10;
    const double duration = 2.0;

    SimpleInverseTendonForceState() :
            tropter::Problem<T>("simple_inverse") {
        this->set_time(0, duration);
        // TODO these functions should return indices for these variables.
        this->add_state("activation", {0, 1});
        this->add_state("norm_tendon_force", {0, 5});
        this->add_control("excitation", {0, 1});
        this->add_control("tendon_force_control", {-50, 50}/*, 0*/);
        this->add_path_constraint("net_generalized_force", 0);
        this->add_path_constraint("fiber_equilibrium", 0);
        m_muscle = DeGrooteFregly2016MuscleStandalone<T>(
                max_isometric_force, optimal_fiber_length, tendon_slack_length,
                pennation_angle_at_optimal, max_contraction_velocity);
    }
    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {
        // TODO compare directly with fiber length state.
        // Unpack variables.
        const T& activation = in.states[0];
        const T& normTenForce = in.states[1];
        const T& excitation = in.controls[0];
        const T& tenForceControl = in.controls[1];

        const T speed = 0.08 / duration;
        const T position = 0.112 + speed * in.time;
        const T netGeneralizedForce = 10 + 10 * in.time / duration;
        //const T position = 0.15 + 0.04 * in.time;
        //const T speed = 0.04;
        //const T netGeneralizedForce = 10 + 5 * in.time;

        // Multibody dynamics.
        // s_F in De Groote, et al. 2016
        const double tendon_force_dynamics_scaling_factor = 10.0;
        const T normTenForceRate =
                tendon_force_dynamics_scaling_factor * tenForceControl;
        T tendonForce;
        T residual;
        // This also computes the fiber equilibrium path constraint.
        m_muscle.calcTendonForceStateEquilibriumResidual(
            activation, position, speed, normTenForce, normTenForceRate,
            residual, tendonForce);

        if (out.path.size() != 0) {
            out.path[0] = tendonForce - netGeneralizedForce;
            out.path[1] = residual;
        }

        // Activation dynamics.
        m_muscle.calcActivationDynamics(excitation, activation,
                out.dynamics[0]);

        // Fiber dynamics.
        out.dynamics[1] = normTenForceRate;
    }
    void calc_integral_cost(const tropter::Input<T>& in, 
            T& integrand) const override {

        const auto& controls = in.controls;
        integrand = controls[0] * controls[0];
    }
private:
    DeGrooteFregly2016MuscleStandalone<T> m_muscle;
};

class SimpleInverseFiberLengthState
        : public tropter::Problem<adouble> {
public:
    using T = adouble;
    const double max_isometric_force = 30;
    const double optimal_fiber_length = 0.10;
    const double tendon_slack_length = 0.05;
    const double pennation_angle_at_optimal = 0.1;
    // optimal fiber lengths per second:
    const double max_contraction_velocity = 10;
    const double duration = 2.0;

    SimpleInverseFiberLengthState() :
            tropter::Problem<T>("simple_inverse") {
        this->set_time(0, duration);
        // TODO these functions should return indices for these variables.
        this->add_state("activation", {0, 1});
        this->add_state("norm_fiber_length", {0.2, 1.8});
        this->add_control("excitation", {0, 1});
        this->add_control("norm_fiber_velocity", {-1, 1}/*, 0*/);
        this->add_path_constraint("net_generalized_force", 0);
        this->add_path_constraint("fiber_equilibrium", 0);
        m_muscle = DeGrooteFregly2016MuscleStandalone<T>(
                max_isometric_force, optimal_fiber_length, tendon_slack_length,
                pennation_angle_at_optimal, max_contraction_velocity);
    }
    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {
        // TODO compare directly with fiber length state.
        // Unpack variables.
        const T& activation = in.states[0];
        const T& normFibLen = in.states[1];
        const T& excitation = in.controls[0];
        const T& normFibVel = in.controls[1];

        const T speed = 0.08 / duration;
        const T position = 0.112 + speed * in.time;
        const T netGeneralizedForce = 10 + 10 * in.time / duration;
        //const T position = 0.15 + 0.04 * in.time;
        //const T speed = 0.04;
        //const T netGeneralizedForce = 10 + 5 * in.time;

        T normTenForce;
        T residual;
        m_muscle.calcEquilibriumResidual(
            activation, position, normFibLen, normFibVel, residual,
            normTenForce);
        T tendonForce = m_muscle.get_max_isometric_force() * normTenForce;
        
        if (out.path.size() != 0) {
            out.path[0] = tendonForce - netGeneralizedForce;
            out.path[1]  = residual;
        }

        // Activation dynamics.
        m_muscle.calcActivationDynamics(excitation, activation,
                out.dynamics[0]);

        // Fiber dynamics.
        out.dynamics[1] = max_contraction_velocity * normFibVel;
    }
    void calc_integral_cost(const tropter::Input<T>& in, 
            T& integrand) const override {

        const auto& controls = in.controls;
        integrand = controls[0] * controls[0];
    }
private:
    DeGrooteFregly2016MuscleStandalone<T> m_muscle;
};

class SimpleInverseTendonForceStateImplicitActivation
        : public tropter::Problem<adouble> {
public:
    using T = adouble;
    const double max_isometric_force = 30;
    const double optimal_fiber_length = 0.10;
    const double tendon_slack_length = 0.05;
    const double pennation_angle_at_optimal = 0.1;
    // optimal fiber lengths per second:
    const double max_contraction_velocity = 10;
    const double duration = 2.0;
    const double activation_time_constant = 0.015;
    const double deactivation_time_constant = 0.060;

    SimpleInverseTendonForceStateImplicitActivation() :
            tropter::Problem<T>("simple_inverse") {
        this->set_time(0, duration);
        // TODO these functions should return indices for these variables.
        this->add_state("activation", {0, 1});
        this->add_state("norm_tendon_force", {0, 5});
        this->add_control("activation_control", {0, 1});
        this->add_control("tendon_force_control", {-50, 50}/*, 0*/);
        this->add_path_constraint("net_generalized_force", 0);
        this->add_path_constraint("fiber_equilibrium", 0);
        this->add_path_constraint("activation_rate_min",
                {0, SimTK::Infinity});
        this->add_path_constraint("activation_rate_max",
                {-SimTK::Infinity, 1.0 / activation_time_constant});
        m_muscle = DeGrooteFregly2016MuscleStandalone<T>(
                max_isometric_force, optimal_fiber_length, tendon_slack_length,
                pennation_angle_at_optimal, max_contraction_velocity);
    }
    // TODO must test implicit form in the case where you want activation to
    // increase as quickly (and decrease as quickly as possible).
    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {
        // TODO compare directly with fiber length state.
        // Unpack variables.
        const T& activation = in.states[0];
        const T& normTenForce = in.states[1];
        const T& activationControl = in.controls[0];
        const T& tenForceControl = in.controls[1];

        const T speed = 0.08 / duration;
        const T position = 0.112 + speed * in.time;
        const T netGeneralizedForce = 10 + 10 * in.time / duration;

        // Multibody dynamics.
        // s_F in De Groote, et al. 2016
        const double tendon_force_dynamics_scaling_factor = 10.0;
        const T normTenForceRate =
                tendon_force_dynamics_scaling_factor * tenForceControl;
        T tendonForce;
        T residual;
        // This also computes the fiber equilibrium path constraint.
        m_muscle.calcTendonForceStateEquilibriumResidual(
                activation, position, speed, normTenForce, normTenForceRate,
                residual, tendonForce);
        // TODO might make more sense to use fiber force; might be a more
        // direct relationship (that, or make tendon length a variable).
        if (out.path.size() != 0) {
            out.path[0] = tendonForce - netGeneralizedForce;
            out.path[1] = residual;
        }

        // Activation dynamics.
        const double activation_rate_min = -1.0 / deactivation_time_constant;
        const double activation_rate_max = 1.0 / activation_time_constant;
        const T activationRate = activation_rate_min
                + (activation_rate_max - activation_rate_min) * activationControl;
        out.dynamics[0] = activationRate;

        if (out.path.size() != 0) {
            out.path[2] = activationRate 
                        + activation / deactivation_time_constant;
            out.path[3] = activationRate 
                        + activation / activation_time_constant;
        }

        // Fiber dynamics.
        out.dynamics[1] = normTenForceRate;
    }
    void calc_integral_cost(const tropter::Input<T>& in,
            T& integrand) const override {

        const auto& states = in.states;
        const auto& controls = in.controls;
        integrand = states[0] * states[0] + 0.01 * controls[0] * controls[0];
    }
private:
    DeGrooteFregly2016MuscleStandalone<T> m_muscle;
};

class SimpleInverseFiberLengthStateImplicitActivation
        : public tropter::Problem<adouble> {
public:
    using T = adouble;
    const double max_isometric_force = 30;
    const double optimal_fiber_length = 0.10;
    const double tendon_slack_length = 0.05;
    const double pennation_angle_at_optimal = 0.1;
    // optimal fiber lengths per second:
    const double max_contraction_velocity = 10;
    const double activation_time_constant = 0.015;
    const double deactivation_time_constant = 0.060;
    const double duration = 2.0;

    SimpleInverseFiberLengthStateImplicitActivation() :
            tropter::Problem<T>("simple_inverse") {
        this->set_time(0, duration);
        // TODO these functions should return indices for these variables.
        this->add_state("activation", {0, 1});
        this->add_state("norm_fiber_length", {0.2, 1.8});
        this->add_control("activation_control", {0, 1});
        this->add_control("norm_fiber_velocity", {-1, 1}/*, 0*/);
        this->add_path_constraint("net_generalized_force", 0);
        this->add_path_constraint("fiber_equilibrium", 0);
        this->add_path_constraint("activation_rate_min",
                {0, SimTK::Infinity});
        this->add_path_constraint("activation_rate_max",
                {-SimTK::Infinity, 1.0 / activation_time_constant});
        m_muscle = DeGrooteFregly2016MuscleStandalone<T>(
                max_isometric_force, optimal_fiber_length, tendon_slack_length,
                pennation_angle_at_optimal, max_contraction_velocity);
    }
    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {
        // TODO compare directly with fiber length state.
        // Unpack variables.
        const T& activation = in.states[0];
        const T& normFibLen = in.states[1];
        const T& activationControl = in.controls[0];
        const T& normFibVel = in.controls[1];

        const T speed = 0.08 / duration;
        const T position = 0.112 + speed * in.time;
        const T netGeneralizedForce = 10 + 10 * in.time / duration;

        T normTenForce;
        T residual;
        m_muscle.calcEquilibriumResidual(
                activation, position, normFibLen, normFibVel, residual,
                normTenForce);
        T tendonForce = m_muscle.get_max_isometric_force() * normTenForce;

        if (out.path.size() != 0) {
            out.path[0] = tendonForce - netGeneralizedForce;
            out.path[1] = residual;
        }

        // Activation dynamics.
        const double activation_rate_min = -1.0 / deactivation_time_constant;
        const double activation_rate_max = 1.0 / activation_time_constant;
        const T activationRate = activation_rate_min
                + (activation_rate_max - activation_rate_min) * activationControl;
        out.dynamics[0] = activationRate;

        if (out.path.size() != 0) {
            out.path[2] = activationRate 
                        + activation / deactivation_time_constant;
            out.path[3] = activationRate 
                        + activation / activation_time_constant;
        }

        // Fiber dynamics.
        out.dynamics[1] = max_contraction_velocity * normFibVel;
    }
    void calc_integral_cost(const tropter::Input<T>& in, 
            T& integrand) const override {

        const auto& states = in.states;
        const auto& controls = in.controls;
        integrand = states[0] * states[0] + 0.01 * controls[0] * controls[0];
    }
private:
    DeGrooteFregly2016MuscleStandalone<T> m_muscle;
};

int main() {
    tropter::Solution tendonForceStateSolution;
    {
        auto ocp = std::make_shared<SimpleInverseTendonForceState>();
        int N = 200;
        tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                "ipopt", N);
        tendonForceStateSolution = dircol.solve();
        tendonForceStateSolution.write("DEBUG_sandboxImplicitActivation.csv");
    }
    tropter::Solution tendonForceStateImpActivSol;
    {
        auto ocp = std::make_shared<
                SimpleInverseTendonForceStateImplicitActivation>();
        int N = 200;
        tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                "ipopt", N);
        tendonForceStateImpActivSol = dircol.solve();
        tendonForceStateImpActivSol.write("DEBUG_sandboxImplicitActivation"
                "_implicitActivation.csv");
    }
    SimTK_TEST(tendonForceStateSolution.states.row(0).isApprox(
            tendonForceStateImpActivSol.states.row(0), 0.01));


    tropter::Solution fiberLengthStateSolution;
    {
        auto ocp = std::make_shared<SimpleInverseFiberLengthState>();
        int N = 200;
        tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                "ipopt", N);
        fiberLengthStateSolution = dircol.solve();
        fiberLengthStateSolution.write(
                "DEBUG_sandboxImplicitActivation_fiberLengthState.csv");
    }
    tropter::Solution fiberLengthStateImpActivSol;
    {
        auto ocp = std::make_shared<
                SimpleInverseFiberLengthStateImplicitActivation>();
        int N = 200;
        tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                "ipopt", N);
        fiberLengthStateImpActivSol = dircol.solve();
        fiberLengthStateImpActivSol.write(
                "DEBUG_sandboxImplicitActivation_fiberLengthState_"
                        "implicitActivation.csv");
    }

    SimTK_TEST(fiberLengthStateSolution.states.row(0).isApprox(
            fiberLengthStateImpActivSol.states.row(0), 0.01));

    // activation_control is the same whether using tendon force or fiber
    // length as a state.
    SimTK_TEST(tendonForceStateImpActivSol.controls.row(0).isApprox(
            fiberLengthStateImpActivSol.controls.row(0), 0.01));


    return EXIT_SUCCESS;
}

