/* -------------------------------------------------------------------------- *
 * OpenSim Moco: test2Muscles1DOFDeGrooteFregly2016.cpp                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco, Christopher Dembia                             *
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
#include "Tests/Testing.h"
#include <Moco/InverseMuscleSolver/DeGrooteFregly2016MuscleStandalone.h>
#include <Moco/InverseMuscleSolver/GlobalStaticOptimization.h>
#include <Moco/InverseMuscleSolver/INDYGO.h>
#include <Moco/InverseMuscleSolver/InverseMuscleSolverMotionData.h>

#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>

#include <tropter/tropter.h>

using namespace OpenSim;

// The objective of this test is to ensure that INDYGO functions properly with
// multiple muscles on a single degree-of-freedom (muscle redundancy problem).
const double ACCEL_GRAVITY = 9.81;

/// Lift two muscles in parallel against gravity from a fixed starting state 
/// to a fixed end position and velocity, in minimum time.
/// Here's a sketch of the problem we solve, without muscle dynamics:
/// @verbatim
///   minimize   t_f
///   subject to qdot = u                     kinematics
///              udot = (mg - f1_t - f2_t)/m  dynamics
///  i = 1,2 -->|fi_t = (a f_l(li_m) f_v(vi_m) + f_p(li_m)) cos(alpha_i)
///              q(0) = q0
///              u(0) = 0
///              q(t_f) = qf
///              u(t_f) = 0
/// @endverbatim
/// where l1_m, 12_m, v1_m and v2_m are determined from the muscle-tendon
/// lengths and velocities with the assumption of rigid tendons.
class DeGrooteFregly2016MusclesLiftMinTimeStatic
    : public tropter::Problem<adouble> {
public:
    using T = adouble;
    const double g = ACCEL_GRAVITY;
    const double mass = 0.5;
    const double max_isometric_force_1 = 30;
    const double max_isometric_force_2 = 30;
    const double optimal_fiber_length_1 = 0.10;
    const double optimal_fiber_length_2 = 0.10;
    const double tendon_slack_length = 0.05;
    const double pennation_angle_at_optimal = 0.1;
    const double max_contraction_velocity = 10;

    DeGrooteFregly2016MusclesLiftMinTimeStatic() :
            tropter::Problem<T>("hanging_muscle_min_time") {
        this->set_time(0, {0.01, 1.0});
        this->add_state("position", {0, 0.3}, 0.15, 0.10);
        this->add_state("speed", {-10, 10}, 0, 0);
        this->add_control("activation_1", {0, 1});
        this->add_control("activation_2", {0, 1});
        this->add_cost("final_time", 0);
        m_muscle_1 = DeGrooteFregly2016MuscleStandalone<T>(
                max_isometric_force_1, optimal_fiber_length_1, 
                tendon_slack_length, pennation_angle_at_optimal,
                max_contraction_velocity);
        m_muscle_2 = DeGrooteFregly2016MuscleStandalone<T>(
                max_isometric_force_2, optimal_fiber_length_2,
                tendon_slack_length, pennation_angle_at_optimal,
                max_contraction_velocity);
    }
    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {
        // Unpack variables.
        const T& position = in.states[0];
        const T& speed = in.states[1];
        const T& activation_1 = in.controls[0];
        const T& activation_2 = in.controls[1];

        // Multibody kinematics.
        out.dynamics[0] = speed;

        // Multibody dynamics.
        const T tendonForce_1 = 
                m_muscle_1.calcRigidTendonFiberForceAlongTendon(activation_1,
                                                                position, 
                                                                speed);
        const T tendonForce_2 = 
                m_muscle_2.calcRigidTendonFiberForceAlongTendon(activation_2,
                                                                position,
                                                                speed); 
        out.dynamics[1] = g - (tendonForce_1 + tendonForce_2) / mass;
    }
    void calc_cost(int cost_index, const tropter::CostInput<adouble>& in,
            T& cost) const override {
        cost = in.final_time;
    }

private:
    DeGrooteFregly2016MuscleStandalone<T> m_muscle_1;
    DeGrooteFregly2016MuscleStandalone<T> m_muscle_2;
};

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectoryGSO() {
    // Solve a trajectory optimization problem.
    // ----------------------------------------
    auto ocp = std::make_shared<DeGrooteFregly2016MusclesLiftMinTimeStatic>();
    ocp->print_description();
    tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt", 100);
    tropter::Solution ocp_solution = dircol.solve();
    std::string trajectoryFile = 
            "test2Muscles1DOFDeGrooteFregly2016_GSO_trajectory.csv";
    ocp_solution.write(trajectoryFile);

    // Save the trajectory with a header so that OpenSim can read it.
    // --------------------------------------------------------------
    // CSVFileAdapter expects an "endheader" line in the file.
    auto fRead = std::ifstream(trajectoryFile);
    std::string trajFileWithHeader = trajectoryFile;
    trajFileWithHeader.replace(trajectoryFile.rfind(".csv"), 4,
                               "_with_header.csv");
    // Skip the "num_states=#", "num_controls=#", "num_adjuncts=#",
    // "num_diffuses=#", and "num_parameters=#" lines.
    std::string line;
    std::getline(fRead, line);
    std::getline(fRead, line);
    std::getline(fRead, line);
    std::getline(fRead, line);
    std::getline(fRead, line);
    auto fWrite = std::ofstream(trajFileWithHeader);
    fWrite << "endheader" << std::endl;
    while (std::getline(fRead, line)) fWrite << line << std::endl;
    fRead.close();
    fWrite.close();

    // Create a table containing only the position and speed of the mass.
    TimeSeriesTable ocpSolution(trajFileWithHeader);
    TimeSeriesTable kinematics;
    kinematics.setColumnLabels({"/joint/height/value",
                                "/joint/height/speed"});
    const auto& position = ocpSolution.getDependentColumn("position");
    const auto& speed = ocpSolution.getDependentColumn("speed");
    for (int iRow = 0; iRow < (int)ocpSolution.getNumRows(); ++iRow) {
        SimTK::RowVector row(2);
        row[0] = position[iRow];
        row[1] = speed[iRow];
        kinematics.appendRow(ocpSolution.getIndependentColumn()[iRow], row);
    }

    // TODO: compute actual inverse dynamics moment, for debugging.
    // TODO: add regression test

    return {ocpSolution, kinematics};
}

/// Lift two muscles in parallel against gravity from a fixed starting state 
/// to a fixed end position and velocity, in minimum time.
/// Here's a sketch of the problem we solve, with activation and fiber dynamics:
/// @verbatim
///   minimize   t_f
///   subject to qdot = u                     kinematics
///              udot = (mg - f1_t - f2_t)/m  dynamics
///  i = 1,2 -->|ai_dot = f_a(ei, ai)         activation dynamics
///             |li_m_dot = vi_m_dot          fiber dynamics
///             |fi_t = (a f_l(li_m) f_v(vi_m) + f_p(li_m)) cos(alpha_i)
///             |vi_m(0) = 0
///              q(0) = q0
///              u(0) = 0
///              a(0) = 0
///              q(t_f) = qf
///              u(t_f) = 0
/// @endverbatim
/// Making the initial fiber velocity 0 helps avoid a sharp spike in fiber
/// velocity at the beginning of the motion.
class DeGrooteFregly2016MusclesLiftMinTimeDynamic
    : public tropter::Problem<adouble> {
public:
    using T = adouble;
    const double g = ACCEL_GRAVITY;
    const double mass = 0.5;
    const double max_isometric_force_1 = 30;
    const double max_isometric_force_2 = 30;
    const double optimal_fiber_length_1 = 0.10;
    const double optimal_fiber_length_2 = 0.10;
    const double tendon_slack_length = 0.05;
    const double pennation_angle_at_optimal = 0.1;
    const double max_contraction_velocity = 10;

    DeGrooteFregly2016MusclesLiftMinTimeDynamic() :
        tropter::Problem<T>("hanging_muscle_min_time") {
        this->set_time(0, {0.01, 1.0});
        this->add_state("position", {0, 0.3}, 0.15, 0.10);
        this->add_state("speed", {-10, 10}, 0, 0);
        this->add_state("activation_1", {0, 1}, 0);
        this->add_state("activation_2", {0, 1}, 0);
        this->add_state("norm_fiber_length_1", {0.2, 1.8});
        this->add_state("norm_fiber_length_2", {0.2, 1.8});
        this->add_control("excitation_1", {0, 1});
        this->add_control("excitation_2", {0, 1});
        this->add_control("norm_fiber_velocity_1", {-1, 1}, 0);
        this->add_control("norm_fiber_velocity_2", {-1, 1}, 0);
        this->add_cost("final_time", 0);
        this->add_path_constraint("fiber_equilibrium_1", 0);
        this->add_path_constraint("fiber_equilibrium_2", 0);
        m_muscle_1 = DeGrooteFregly2016MuscleStandalone<T>(
            max_isometric_force_1, optimal_fiber_length_1,
            tendon_slack_length, pennation_angle_at_optimal,
            max_contraction_velocity);
        m_muscle_2 = DeGrooteFregly2016MuscleStandalone<T>(
            max_isometric_force_2, optimal_fiber_length_2,
            tendon_slack_length, pennation_angle_at_optimal,
            max_contraction_velocity);
    }
    void calc_differential_algebraic_equations(
        const tropter::Input<T>& in,
        tropter::Output<T> out) const override {
        // Unpack variables.
        const T& position = in.states[0];
        const T& speed = in.states[1];
        const T& activation_1 = in.states[2];
        const T& activation_2 = in.states[3];
        const T& normFibLen_1 = in.states[4];
        const T& normFibLen_2 = in.states[5];
        const T& excitation_1 = in.controls[0];
        const T& excitation_2 = in.controls[1];
        const T& normFibVel_1 = in.controls[2];
        const T& normFibVel_2 = in.controls[3];

        // Multibody kinematics.
        out.dynamics[0] = speed;

        // Multibody dynamics.
        T normTenForce_1;
        T normTenForce_2;
        T residual_1;
        T residual_2;
        m_muscle_1.calcEquilibriumResidual(
                activation_1, position, normFibLen_1, normFibVel_1, 
                residual_1, normTenForce_1);
        m_muscle_2.calcEquilibriumResidual(
                activation_2, position, normFibLen_2, normFibVel_2,
                residual_2, normTenForce_2);
        if (out.path.size() != 0) {
            out.path[0] = residual_1;
            out.path[1] = residual_2;
        }
        T tendonForce_1 = m_muscle_1.get_max_isometric_force()*normTenForce_1;
        T tendonForce_2 = m_muscle_2.get_max_isometric_force()*normTenForce_2;
        out.dynamics[1] = g - (tendonForce_1 + tendonForce_2) / mass;

        // Activation dynamics.
        m_muscle_1.calcActivationDynamics(excitation_1, activation_1,
                out.dynamics[2]);
        m_muscle_2.calcActivationDynamics(excitation_2, activation_2,
                out.dynamics[3]);

        // Fiber dynamics.
        out.dynamics[4] = max_contraction_velocity * normFibVel_1;
        out.dynamics[5] = max_contraction_velocity * normFibVel_2;
    }
    void calc_cost(int cost_index, const tropter::CostInput<adouble>& in,
            T& cost) const override {
        cost = in.final_time;
    }
private:
    DeGrooteFregly2016MuscleStandalone<T> m_muscle_1;
    DeGrooteFregly2016MuscleStandalone<T> m_muscle_2;
};

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectoryINDYGO() {
    // Solve a trajectory optimization problem.
    // ----------------------------------------
    auto ocp = std::make_shared<DeGrooteFregly2016MusclesLiftMinTimeDynamic>();
    ocp->print_description();
    tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
        "ipopt", 100);
    // The quasi-Newton method gives a pretty good speedup for this problem.
    dircol.get_opt_solver().set_hessian_approximation("limited-memory");
    tropter::Solution ocp_solution = dircol.solve();
    std::string trajectoryFile =
        "test2Muscles1DOFDeGrooteFregly2016_INDYGO_trajectory.csv";
    ocp_solution.write(trajectoryFile);

    // Save the trajectory with a header so that OpenSim can read it.
    // --------------------------------------------------------------
    // CSVFileAdapter expects an "endheader" line in the file.
    auto fRead = std::ifstream(trajectoryFile);
    std::string trajFileWithHeader = trajectoryFile;
    trajFileWithHeader.replace(trajectoryFile.rfind(".csv"), 4,
        "_with_header.csv");
    // Skip the "num_states=#", "num_controls=#", "num_adjuncts=#",
    // "num_diffus=#", and "num_parameters=#" lines.
    std::string line;
    std::getline(fRead, line);
    std::getline(fRead, line);
    std::getline(fRead, line);
    std::getline(fRead, line);
    std::getline(fRead, line);
    auto fWrite = std::ofstream(trajFileWithHeader);
    fWrite << "endheader" << std::endl;
    while (std::getline(fRead, line)) fWrite << line << std::endl;
    fRead.close();
    fWrite.close();

    // Create a table containing only the position and speed of the mass.
    TimeSeriesTable ocpSolution(trajFileWithHeader);
    TimeSeriesTable kinematics;
    kinematics.setColumnLabels({ "/joint/height/value",
        "/joint/height/speed" });
    const auto& position = ocpSolution.getDependentColumn("position");
    const auto& speed = ocpSolution.getDependentColumn("speed");
    for (int iRow = 0; iRow < (int)ocpSolution.getNumRows(); ++iRow) {
        SimTK::RowVector row(2);
        row[0] = position[iRow];
        row[1] = speed[iRow];
        kinematics.appendRow(ocpSolution.getIndependentColumn()[iRow], row);
    }

    return{ocpSolution, kinematics};
}

OpenSim::Model buildLiftingMassModel() {
    DeGrooteFregly2016MusclesLiftMinTimeDynamic ocp;
    Model model;
    model.setName("hanging_muscles");

    // First, we need to build a similar OpenSim model.
    model.set_gravity(SimTK::Vec3(ocp.g, 0, 0));
    auto* body = new Body("body", ocp.mass,
        SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint();
    joint->setName("joint");
    joint->connectSocket_parent_frame(model.getGround());
    joint->connectSocket_child_frame(*body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("height");
    model.addComponent(joint);

    auto* actu_1 = new Millard2012EquilibriumMuscle();
    actu_1->setName("muscle_1");
    actu_1->set_max_isometric_force(ocp.max_isometric_force_1);
    actu_1->set_optimal_fiber_length(ocp.optimal_fiber_length_1);
    actu_1->set_tendon_slack_length(ocp.tendon_slack_length);
    actu_1->set_pennation_angle_at_optimal(ocp.pennation_angle_at_optimal);
    actu_1->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu_1->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addComponent(actu_1);

    auto* actu_2 = new Millard2012EquilibriumMuscle();
    actu_2->setName("muscle_2");
    actu_2->set_max_isometric_force(ocp.max_isometric_force_2);
    actu_2->set_optimal_fiber_length(ocp.optimal_fiber_length_2);
    actu_2->set_tendon_slack_length(ocp.tendon_slack_length);
    actu_2->set_pennation_angle_at_optimal(ocp.pennation_angle_at_optimal);
    actu_2->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu_2->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addComponent(actu_2);

    model.finalizeConnections();
    return model;
}

void testLiftingMassGSO(
    const std::pair<TimeSeriesTable, TimeSeriesTable>& data) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    // Build a similar OpenSim model.
    // ------------------------------
    Model model = buildLiftingMassModel();
    model.finalizeFromProperties();

    // Create the GlobalStaticOptimization.
    // ------------------------------------------
    GlobalStaticOptimization gso;
    gso.setModel(model);
    gso.setKinematicsData(kinematics);
    gso.set_lowpass_cutoff_frequency_for_joint_moments(100);
    gso.set_create_reserve_actuators(0.001);
    gso.set_mesh_point_frequency(800);
    GlobalStaticOptimization::Solution solution = gso.solve();
    solution.write("test2Muscles1DOFDeGrooteFregly2016_GSO");

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------

    // The rationale for the tolerances: as tight as they could be for the
    // test to pass.
    rootMeanSquare(solution.activation, "/muscle_1",
        ocpSolution, "activation_1", 
        0.05);
    rootMeanSquare(solution.activation, "/muscle_2",
        ocpSolution, "activation_2", 
        0.05);
}

// Reproduce the trajectory using INDYGO
void testLiftingMassINDYGO(
    const std::pair<TimeSeriesTable, TimeSeriesTable>& data,
    bool useStaticOptiGuess) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    // Build a similar OpenSim model.
    // ------------------------------
    Model model = buildLiftingMassModel();
    model.finalizeFromProperties();

    // Create the INDYGO.
    // ----------------------------------
    INDYGO mrs;
    mrs.setModel(model);
    mrs.setKinematicsData(kinematics);
    // Without filtering, the moments have high frequency content,
    // probably related to unfiltered generalized coordinates and getting
    // accelerations from a spline fit.
    mrs.set_lowpass_cutoff_frequency_for_joint_moments(80);
    mrs.set_create_reserve_actuators(0.001);
    if (useStaticOptiGuess) {
        mrs.set_initial_guess("static_optimization");
    }
    else {
        mrs.set_initial_guess("bounds");
    }
    INDYGO::Solution solution = mrs.solve();
    solution.write("test2Muscles1DOFDeGrooteFregly2016_INDYGO");

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------

    rootMeanSquare(solution.activation, "/muscle_1",
        ocpSolution, "activation_1",
        0.04);
    rootMeanSquare(solution.activation, "/muscle_2",
        ocpSolution, "activation_2",
        0.04);
    compare(solution.norm_fiber_length, "/muscle_1",
        ocpSolution, "norm_fiber_length_1",
        0.005);
    compare(solution.norm_fiber_length, "/muscle_2",
        ocpSolution, "norm_fiber_length_2",
        0.005);

    // We use a weaker check for the controls; they don't match as well.
    rootMeanSquare(solution.excitation, "/muscle_1",
        ocpSolution, "excitation_1",
        0.25);
    rootMeanSquare(solution.excitation, "/muscle_2",
        ocpSolution, "excitation_2",
        0.25);
    rootMeanSquare(solution.norm_fiber_velocity, "/muscle_1",
        ocpSolution, "norm_fiber_velocity_1",
        0.03);
    rootMeanSquare(solution.norm_fiber_velocity, "/muscle_2",
        ocpSolution, "norm_fiber_velocity_2",
        0.03);
}

int main() {

    SimTK_START_TEST("test2Muscles1DOFDeGrooteFregly2016");

        {
            auto gsoData = solveForTrajectoryGSO();
            SimTK_SUBTEST1(testLiftingMassGSO, gsoData);
        }
        {
            auto mrsData = solveForTrajectoryINDYGO();
            // Without using static optimization to obtain an initial guess.
            SimTK_SUBTEST2(testLiftingMassINDYGO, mrsData, false);
            // Use static optimization to obtain an initial guess.
            SimTK_SUBTEST2(testLiftingMassINDYGO, mrsData, true);
        }
        
    SimTK_END_TEST();
   
}
