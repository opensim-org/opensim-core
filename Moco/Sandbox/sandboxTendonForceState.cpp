/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxTendonForceState.cpp                                  *
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

// TODO not getting the correct solution yet (switching time is too early,
// the final time is longer than with the fiber length state variable).

class DeGrooteFregly2016MuscleLiftMinTimeDynamic
        : public tropter::Problem<adouble> {
public:
    using T = adouble;
    const double g = 9.81;
    const double mass = 0.5;
    const double max_isometric_force = 30;
    const double optimal_fiber_length = 0.10;
    const double tendon_slack_length = 0.05;
    const double pennation_angle_at_optimal = 0.1;
    // optimal fiber lengths per second:
    const double max_contraction_velocity = 10;

    DeGrooteFregly2016MuscleLiftMinTimeDynamic() :
            tropter::Problem<T>("hanging_muscle_min_time") {
        this->set_time(0, {0.01, 1.0});
        // TODO these functions should return indices for these variables.
        this->add_state("position", {0, 0.3}, 0.15, 0.10);
        this->add_state("speed", {-10, 10}, 0, 0);
        this->add_state("activation", {0, 1}, 0);
        this->add_state("norm_tendon_force", {0, 5});
        this->add_control("excitation", {0, 1});
        this->add_control("tendon_force_control", {-50, 50}/*, 0*/);
        this->add_path_constraint("fiber_equilibrium", 0);
        m_muscle = DeGrooteFregly2016MuscleStandalone<T>(
                max_isometric_force, optimal_fiber_length, tendon_slack_length,
                pennation_angle_at_optimal, max_contraction_velocity);
    }
    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {
        // Unpack variables.
        const T& position = in.states[0];
        const T& speed = in.states[1];
        const T& activation = in.states[2];
        const T& normTenForce = in.states[3];
        const T& excitation = in.controls[0];
        const T& tenForceControl = in.controls[1];

        // Multibody kinematics.
        out.dynamics[0] = speed;

        // Multibody dynamics.
        // s_F in De Groote, et al. 2016
        const double tendon_force_dynamics_scaling_factor = 10.0;
        const T normTenForceRate =
                tendon_force_dynamics_scaling_factor * tenForceControl;
        T tendonForce;
        // This also computes the fiber equilibrium path constraint.
        T residual;
        m_muscle.calcTendonForceStateEquilibriumResidual(
                activation, position, speed, normTenForce, normTenForceRate,
                residual, tendonForce);
        if (out.path.size() != 0) {
            out.path[0] = residual;
        }
        // TODO might make more sense to use fiber force; might be a more
        // direct relationship (that, or make tendon length a variable).
        out.dynamics[1] = g - tendonForce / mass;

        // Activation dynamics.
        m_muscle.calcActivationDynamics(excitation, activation,
                out.dynamics[2]);

        // Fiber dynamics.
        out.dynamics[3] = normTenForceRate;
    }
    void calc_endpoint_cost(const T& final_time,
            const tropter::VectorX<T>& /*final_states*/,
            const tropter::VectorX<T>& /*parameters*/,
            T& cost) const override {
        cost = final_time;
    }
private:
    DeGrooteFregly2016MuscleStandalone<T> m_muscle;
};

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectoryINDYGO() {
    // Solve a trajectory optimization problem.
    // ----------------------------------------
    auto ocp = std::make_shared<DeGrooteFregly2016MuscleLiftMinTimeDynamic>();
    ocp->print_description();
    tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
            "ipopt", 100);
    // The quasi-Newton method gives a pretty good speedup for this problem.
    dircol.get_opt_solver().set_hessian_approximation("limited-memory");
    tropter::Solution ocp_solution = dircol.solve();
    dircol.print_constraint_values(ocp_solution);
    std::string trajectoryFile =
            "sandboxTendonForceState_INDYGO_trajectory.csv";
    ocp_solution.write(trajectoryFile);

    // Save the trajectory with a header so that OpenSim can read it.
    // --------------------------------------------------------------
    // CSVFileAdapter expects an "endheader" line in the file.
    auto fRead = std::ifstream(trajectoryFile);
    std::string trajFileWithHeader = trajectoryFile;
    trajFileWithHeader.replace(trajectoryFile.rfind(".csv"), 4,
            "_with_header.csv");
    // Skip the "num_states=#" and "num_controls=#" lines.
    std::string line;
    std::getline(fRead, line);
    std::getline(fRead, line);
    auto fWrite = std::ofstream(trajFileWithHeader);
    fWrite << "endheader" << std::endl;
    while (std::getline(fRead, line)) fWrite << line << std::endl;
    fRead.close();
    fWrite.close();

    // Create a table containing only the position and speed of the mass.
    TimeSeriesTable ocpSolution = CSVFileAdapter::read(trajFileWithHeader);
    TimeSeriesTable kinematics;
    kinematics.setColumnLabels({"joint/height/value",
                                "joint/height/speed"});
    const auto& position = ocpSolution.getDependentColumn("position");
    const auto& speed = ocpSolution.getDependentColumn("speed");
    for (int iRow = 0; iRow < (int)ocpSolution.getNumRows(); ++iRow) {
        SimTK::RowVector row(2);
        row[0] = position[iRow];
        row[1] = speed[iRow];
        kinematics.appendRow(ocpSolution.getIndependentColumn()[iRow], row);
    }

    // Compute actual inverse dynamics moment, for debugging.
    // ------------------------------------------------------
    // TimeSeriesTable actualInvDyn;
    // actualInvDyn.setColumnLabels({"inverse_dynamics"});
    // DeGrooteFregly2016MuscleStandalone<double> muscle(ocp->max_isometric_force,
    //                                   ocp->optimal_fiber_length,
    //                                   ocp->tendon_slack_length,
    //                                   ocp->pennation_angle_at_optimal,
    //                                   ocp->max_contraction_velocity);
    // for (Eigen::Index iTime = 0; iTime < ocp_solution.time.size(); ++iTime) {
    //     const auto& musTenLength = ocp_solution.states(0, iTime);
    //     const auto& normFiberLength = ocp_solution.states(3, iTime);
    //     double tendonForce;
    //     muscle.calcTendonForce(musTenLength, normFiberLength, tendonForce);
    //     actualInvDyn.appendRow(ocp_solution.time(iTime),
    //                            SimTK::RowVector(1, -tendonForce));
    // }
    // CSVFileAdapter::write(actualInvDyn,
    //                       "DEBUG_testLiftingMass_INDYGO_actualInvDyn.csv");

    return {ocpSolution, kinematics};
}

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
        // TODO might make more sense to use fiber force; might be a more
        // direct relationship (that, or make tendon length a variable).
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
            out.path[1] = residual;
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

int main() {
    //solveForTrajectoryINDYGO();
    tropter::Solution tendonForceStateSolution;
    {
        auto ocp = std::make_shared<SimpleInverseTendonForceState>();
        int N = 100;
        tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                "ipopt", N);
        tendonForceStateSolution = dircol.solve();
        tendonForceStateSolution.write("DEBUG_sandboxTendonForceState.csv");
    }
    tropter::Solution fiberLengthStateSolution;
    {
        auto ocp = std::make_shared<SimpleInverseFiberLengthState>();
        int N = 100;
        tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                "ipopt", N);
        fiberLengthStateSolution = dircol.solve();
        fiberLengthStateSolution.write(
                "DEBUG_sandboxTendonForceState_fiberLengthState.csv");
    }

    // Activation.

//    std::cout << "DEUBG " << tendonForceStateSolution.states.row(0) -
//    fiberLengthStateSolution.states.row(0) << std::endl;
    SimTK_TEST(tendonForceStateSolution.states.row(0).isApprox(
                    fiberLengthStateSolution.states.row(0), 0.02));

    SimTK_TEST(tendonForceStateSolution.controls.row(0).isApprox(
            fiberLengthStateSolution.controls.row(0), 0.02));
}
