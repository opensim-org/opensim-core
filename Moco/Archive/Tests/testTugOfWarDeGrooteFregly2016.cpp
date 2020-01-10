/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testTugOfWarDeGrooteFregly2016.cpp                           *
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
#include "Tests/Testing.h"
#include <Moco/InverseMuscleSolver/DeGrooteFregly2016MuscleStandalone.h>
#include <Moco/InverseMuscleSolver/GlobalStaticOptimization.h>
#include <Moco/InverseMuscleSolver/INDYGO.h>

#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>

#include <tropter/tropter.h>

using namespace OpenSim;

// The objective of this test is to ensure that INDYGO functions properly with
// multiple muscles that oppose each other.

const double DISTANCE = 0.25;

// TODO make displacements larger.

/// Two muscles coordinate to move a mass on a table (no gravity) from one
/// fixed state to another, with minimum effort.
///
///                            mass
///                |------------O--------------|
///                   muscle L      muscle R
///
///                 <- - d- - ->|-> q
///
/// Here's a sketch of the problem we solve, without activation or fiber
/// dynamics.
/// @verbatim
///   minimize   int_t (aL^2 + aR^2) dt
///   subject to qdot = u                  kinematics
///              udot = 1/m (-f_tL + f_tR) dynamics
///              f_tL = (aL f_l(lmL) f_v(vmL) + f_p(lmL)) cos(alphaL)
///              f_tR = (aR f_l(lmR) f_v(vmR) + f_p(lmR)) cos(alphaR)
///              q(0) = -0.015
///              u(0) = 0
///              aL(0) = 0
///              aR(0) = 0
///              q(t_f) = 0.015
///              u(t_f) = 0
/// @endverbatim
template <typename T>
class DeGrooteFregly2016MuscleTugOfWarMinEffortStatic
        : public tropter::Problem<T> {
public:
    const double d = DISTANCE;
    double mass = -1;
    int m_i_position = -1;
    int m_i_speed = -1;
    int m_i_activation_l = -1;
    int m_i_activation_r = -1;
    DeGrooteFregly2016MuscleStandalone<T> m_muscleL;
    DeGrooteFregly2016MuscleStandalone<T> m_muscleR;

    DeGrooteFregly2016MuscleTugOfWarMinEffortStatic(const Model& model) :
            tropter::Problem<T>("tug_of_war_min_effort") {
        this->set_time(0, 0.5);
        m_i_position =
                this->add_state("position", {-0.02, 0.02}, -0.015, 0.015);
        m_i_speed = this->add_state("speed", {-5, 15}, 0, 0);
        m_i_activation_l = this->add_control("activation_l", {0, 1});
        m_i_activation_r = this->add_control("activation_r", {0, 1});
        this->add_cost("effort", 1);
        mass = dynamic_cast<const Body&>(model.getComponent("body")).get_mass();
        {
            const auto& osimMuscleL =
                    dynamic_cast<const Muscle&>(model.getComponent("left"));
            m_muscleL = DeGrooteFregly2016MuscleStandalone<T>(
                    osimMuscleL.get_max_isometric_force(),
                    osimMuscleL.get_optimal_fiber_length(),
                    osimMuscleL.get_tendon_slack_length(),
                    osimMuscleL.get_pennation_angle_at_optimal(),
                    osimMuscleL.get_max_contraction_velocity());
        }
        {
            const auto& osimMuscleR =
                    dynamic_cast<const Muscle&>(model.getComponent("right"));
            m_muscleR = DeGrooteFregly2016MuscleStandalone<T>(
                    osimMuscleR.get_max_isometric_force(),
                    osimMuscleR.get_optimal_fiber_length(),
                    osimMuscleR.get_tendon_slack_length(),
                    osimMuscleR.get_pennation_angle_at_optimal(),
                    osimMuscleR.get_max_contraction_velocity());
        }
    }
    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {
        // Unpack variables.
        // -----------------
        const T& speed = in.states[m_i_speed];

        // Multibody kinematics.
        // ---------------------
        out.dynamics[m_i_position] = speed;

        // Multibody dynamics.
        // -------------------
        const T netForce = calcNetForce(in.states, in.controls);
        out.dynamics[m_i_speed] = netForce / mass;
    }
    T calcNetForce(const Eigen::Ref<const tropter::VectorX<T>>& states,
            const Eigen::Ref<const tropter::VectorX<T>>& controls) const {
        const T& position = states[m_i_position];
        const T& speed = states[m_i_speed];

        const T& activationL = controls[m_i_activation_l];
        const T forceL = m_muscleL.calcRigidTendonFiberForceAlongTendon(
                activationL, d + position, speed);

        const T& activationR = controls[m_i_activation_r];
        const T forceR = m_muscleR.calcRigidTendonFiberForceAlongTendon(
                activationR, d - position, -speed);

        return -forceL + forceR;
    }
    void calc_cost_integrand(int cost_index, const tropter::Input<T>& in,
            T& integrand) const override {
        const auto& controls = in.controls;
        const auto& controlL = controls[m_i_activation_l];
        const auto& controlR = controls[m_i_activation_r];
        integrand = controlL * controlL + controlR * controlR;
    }
    void calc_cost(int cost_index, const tropter::CostInput<T>& in,
            T& cost) const override {
        cost = in.integral;
    }
};

/// Two muscles coordinate to move a mass on a table (no gravity) from one
/// fixed state to another, with minimum effort.
///
///                            mass
///                |------------O--------------|
///                   muscle L      muscle R
///
///                 <- - d- - ->|-> q
///
/// Here's a sketch of the problem we solve, with activation and fiber dynamics.
/// @verbatim
///   minimize   int_t (eL^2 + eR^2) dt
///   subject to qdot = u                  kinematics
///              udot = 1/m (-f_tL + f_tR) dynamics
///              aLdot = f_a(eL, aL)       activation dynamics
///              aRdot = f_a(eR, aR)
///              lmLdot = vmLdot           fiber dynamics
///              lmRdot = vmRdot
///(for L and R) (a f_l(lm) f_v(vm) + f_p(lm)) cos(alpha) = f_t(lt) equilibrium
///              q(0) = -0.15
///              u(0) = 0
///              aL(0) = 0
///              aR(0) = 0
///              vmL(0) = 0
///              vmR(0) = 0
///              q(t_f) = 0.15
///              u(t_f) = 0
/// @endverbatim
// Including pennation seems to really slow down the problem. perhaps
// first solve the problem without pennation?
// Activation dynamics do *not* substantially slow down the solution process.
template <typename T>
class DeGrooteFregly2016MuscleTugOfWarMinEffortDynamic
        : public tropter::Problem<T> {
public:
    const double d = DISTANCE;
    double mass = -1;
    int m_i_position = -1;
    int m_i_speed = -1;
    int m_i_activation_l = -1;
    int m_i_activation_r = -1;
    int m_i_norm_fiber_length_l = -1;
    int m_i_norm_fiber_length_r = -1;
    int m_i_excitation_l = -1;
    int m_i_excitation_r = -1;
    int m_i_norm_fiber_velocity_l = -1;
    int m_i_fiber_equilibrium_l = -1;
    int m_i_norm_fiber_velocity_r = -1;
    int m_i_fiber_equilibrium_r = -1;

    DeGrooteFregly2016MuscleStandalone<T> m_muscleL;
    DeGrooteFregly2016MuscleStandalone<T> m_muscleR;

    DeGrooteFregly2016MuscleTugOfWarMinEffortDynamic(const Model& model) :
            tropter::Problem<T>("tug_of_war_min_effort") {
        this->set_time(0, 0.5);
        m_i_position =
                this->add_state("position", {-0.02, 0.02}, -0.015, 0.015);
        m_i_speed = this->add_state("speed", {-5, 15}, 0, 0);
        m_i_activation_l = this->add_state("activation_l", {0, 1}, 0);
        m_i_activation_r = this->add_state("activation_r", {0, 1}, 0);
        m_i_norm_fiber_length_l =
                this->add_state("norm_fiber_length_l", {0.2, 1.8});
        m_i_norm_fiber_length_r =
                this->add_state("norm_fiber_length_r", {0.2, 1.8});
        m_i_excitation_l = this->add_control("excitation_l", {0, 1});
        m_i_excitation_r = this->add_control("excitation_r", {0, 1});
        m_i_norm_fiber_velocity_l =
                this->add_control("norm_fiber_velocity_l", {-1, 1}, 0);
        m_i_norm_fiber_velocity_r =
                this->add_control("norm_fiber_velocity_r", {-1, 1}, 0);
        this->add_cost("effort", 1);
        m_i_fiber_equilibrium_l =
                this->add_path_constraint("fiber_equilibrium_l", 0);
        m_i_fiber_equilibrium_r =
                this->add_path_constraint("fiber_equilibrium_r", 0);

        mass = dynamic_cast<const Body&>(model.getComponent("body")).get_mass();
        {
            const auto& osimMuscleL =
                    dynamic_cast<const Muscle&>(model.getComponent("left"));
            m_muscleL = DeGrooteFregly2016MuscleStandalone<T>(
                    osimMuscleL.get_max_isometric_force(),
                    osimMuscleL.get_optimal_fiber_length(),
                    osimMuscleL.get_tendon_slack_length(),
                    osimMuscleL.get_pennation_angle_at_optimal(),
                    osimMuscleL.get_max_contraction_velocity());
        }
        {
            const auto& osimMuscleR =
                    dynamic_cast<const Muscle&>(model.getComponent("right"));
            m_muscleR = DeGrooteFregly2016MuscleStandalone<T>(
                    osimMuscleR.get_max_isometric_force(),
                    osimMuscleR.get_optimal_fiber_length(),
                    osimMuscleR.get_tendon_slack_length(),
                    osimMuscleR.get_pennation_angle_at_optimal(),
                    osimMuscleR.get_max_contraction_velocity());
        }
    }
    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {
        const auto& states = in.states;
        const auto& controls = in.controls;
        // Unpack variables.
        // -----------------
        const T& speed = states[m_i_speed];

        // Multibody kinematics.
        // ---------------------
        out.dynamics[m_i_position] = speed;

        // Multibody dynamics.
        // -------------------
        const T netForce = calcNetForce(states);

        out.dynamics[m_i_speed] = netForce / mass;

        // Activation dynamics.
        // --------------------
        const T& activationL = states[m_i_activation_l];
        const T& excitationL = controls[m_i_excitation_l];
        m_muscleL.calcActivationDynamics(excitationL, activationL,
                                         out.dynamics[m_i_activation_l]);
        const T& activationR = states[m_i_activation_r];
        const T& excitationR = controls[m_i_excitation_r];
        m_muscleR.calcActivationDynamics(excitationR, activationR,
                                         out.dynamics[m_i_activation_r]);

        // Fiber dynamics.
        // ---------------
        const T& normFibVelL = controls[m_i_norm_fiber_velocity_l];
        const T& normFibVelR = controls[m_i_norm_fiber_velocity_r];
        out.dynamics[m_i_norm_fiber_length_l] =
                m_muscleL.get_max_contraction_velocity() * normFibVelL;
        out.dynamics[m_i_norm_fiber_length_r] =
                m_muscleR.get_max_contraction_velocity() * normFibVelR;

        // Path constraints.
        // =================
        if (out.path.size() != 0) {
            const T& position = states[m_i_position];
            {
                const T& activationL = states[m_i_activation_l];
                const T& normFibLenL = states[m_i_norm_fiber_length_l];
                const T& normFibVelL = controls[m_i_norm_fiber_velocity_l];
                m_muscleL.calcEquilibriumResidual(activationL, d + position,
                        normFibLenL, normFibVelL,
                        out.path[m_i_fiber_equilibrium_l]);
            }
            {
                const T& activationR = states[m_i_activation_r];
                const T& normFibLenR = states[m_i_norm_fiber_length_r];
                const T& normFibVelR = controls[m_i_norm_fiber_velocity_r];
                m_muscleR.calcEquilibriumResidual(activationR, d - position,
                        normFibLenR, normFibVelR,
                        out.path[m_i_fiber_equilibrium_r]);
            }
        }
    }
    T calcNetForce(const Eigen::Ref<const tropter::VectorX<T>>& states) const {
        const T& position = states[m_i_position];

        T forceL;
        const T& normFibLenL = states[m_i_norm_fiber_length_l];
        m_muscleL.calcTendonForce(d + position, normFibLenL, forceL);
        T forceR;
        const T& normFibLenR = states[m_i_norm_fiber_length_r];
        m_muscleR.calcTendonForce(d - position, normFibLenR, forceR);

        return -forceL + forceR;
    }
    void calc_cost_integrand(int cost_index, const tropter::Input<T>& in,
            T& integrand) const override {
        const auto& controls = in.controls;
        const auto& controlL = controls[m_i_excitation_l];
        const auto& controlR = controls[m_i_excitation_r];
        integrand = controlL * controlL + controlR * controlR;
    }
    void calc_cost(int cost_index, const tropter::CostInput<T>& in,
            T& cost) const override {
        cost = in.integral;
    }
};

OpenSim::Model buildTugOfWarModel() {
    using SimTK::Vec3;

    Model model;
    // model.setUseVisualizer(true);
    model.setName("tug_of_war");

    model.set_gravity(Vec3(0));
    auto* body = new Body("body", 1, Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint();
    joint->setName("joint");
    joint->connectSocket_parent_frame(model.getGround());
    joint->connectSocket_child_frame(*body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model.addComponent(joint);

    // The muscles' opt fib len + tendon slack length is slightly greater
    // than d.
    {
        auto* actuL = new Millard2012EquilibriumMuscle();
        actuL->setName("left");
        actuL->set_max_isometric_force(2);
        actuL->set_optimal_fiber_length(.20);
        actuL->set_tendon_slack_length(0.05);
        // Pennation=0 speeds up convergence (126 iterations -> 52 iterations).
        actuL->set_pennation_angle_at_optimal(0.0);
        actuL->addNewPathPoint("origin", model.updGround(),
                               Vec3(-DISTANCE, 0, 0));
        actuL->addNewPathPoint("insertion", *body, Vec3(0));
        model.addComponent(actuL);
    }
    {
        auto* actuR = new Millard2012EquilibriumMuscle();
        actuR->setName("right");
        actuR->set_max_isometric_force(2);
        actuR->set_optimal_fiber_length(.15);
        actuR->set_tendon_slack_length(0.10);
        // Pennation=0 speeds up convergence (126 iterations -> 52 iterations).
        actuR->set_pennation_angle_at_optimal(0.0);
        actuR->addNewPathPoint("origin", model.updGround(),
                               Vec3(DISTANCE, 0, 0));
        actuR->addNewPathPoint("insertion", *body, Vec3(0));
        model.addComponent(actuR);
    }

    // SimTK::State s = model.initSystem();
    // model.updMatterSubsystem().setShowDefaultGeometry(true);
    // model.updVisualizer().updSimbodyVisualizer().setBackgroundType(
    //         SimTK::Visualizer::GroundAndSky);
    // model.getVisualizer().show(s);
    // std::cin.get();
    // Manager manager(model);
    // manager.integrate(s, 1.0);

    model.finalizeConnections();
    return model;
}

template <typename T>
using TugOfWarStatic = DeGrooteFregly2016MuscleTugOfWarMinEffortStatic<T>;

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectory_GSO(const Model& model) {

    // Solve a trajectory optimization problem.
    // ----------------------------------------

    auto ocp = std::make_shared<TugOfWarStatic<adouble>>(model);
    ocp->print_description();
    const int N = 100;
    tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt", N);
    tropter::Solution ocp_solution = dircol.solve();
    std::string trajectoryFile =
            "testTugOfWarDeGrooteFregly2016_GSO_trajectory.csv";
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

    // Create a table containing only the angle and speed of the pendulum.
    TimeSeriesTable ocpSolution(trajFileWithHeader);
    TimeSeriesTable kinematics;
    kinematics.setColumnLabels({"/joint/position/value",
                                "/joint/position/speed"});
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
    TimeSeriesTable actualInvDyn;
    actualInvDyn.setColumnLabels({"inverse_dynamics"});
    auto ocpd = std::make_shared<TugOfWarStatic<double>>(model);
    for (Eigen::Index iTime = 0; iTime < ocp_solution.time.size(); ++iTime) {
        auto netForce = ocpd->calcNetForce(ocp_solution.states.col(iTime),
                                           ocp_solution.controls.col(iTime));
        actualInvDyn.appendRow(ocp_solution.time(iTime),
                               SimTK::RowVector(1, netForce));
    }
    CSVFileAdapter::write(actualInvDyn,
        "DEBUG_testTugOfWar_GSO_actualInvDyn.csv");

    return {ocpSolution, kinematics};
}

template <typename T>
using TugOfWarDynamic = DeGrooteFregly2016MuscleTugOfWarMinEffortDynamic<T>;

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectory_INDYGO(const Model& model) {
    // Solve a trajectory optimization problem.
    // ----------------------------------------
    auto ocp = std::make_shared<TugOfWarDynamic<adouble>>(model);
    ocp->print_description();
    const int N = 100;
    tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt", N);
    // The quasi-Newton method gives a pretty good speedup for this problem.
    dircol.get_opt_solver().set_hessian_approximation("limited-memory");
    // Create an initial guess.
    // This initial guess reduces the number of iterations from 52 to 32.
    // using Eigen::RowVectorXd;
    // tropter::Iterate guess;
    // guess.time.setLinSpaced(N, 0, 0.5);
    // ocp->set_state_guess(guess, "position",
    //                      RowVectorXd::LinSpaced(N, -0.015, 0.015));
    // ocp->set_state_guess(guess, "speed", RowVectorXd::Constant(N, 0.06));
    // ocp->set_state_guess(guess, "activation_l",
    //                      RowVectorXd::LinSpaced(N, 0, 1));
    // ocp->set_state_guess(guess, "norm_fiber_length_l",
    //                      RowVectorXd::LinSpaced(N, 0.9, 1.1));
    // ocp->set_control_guess(guess, "excitation_l",
    //                        RowVectorXd::LinSpaced(N, 0, 1));
    // ocp->set_control_guess(guess, "norm_fiber_velocity_l",
    //                        RowVectorXd::Constant(N, 0.03));
    // ocp->set_state_guess(guess, "activation_r",
    //                      RowVectorXd::LinSpaced(N, 1, 0));
    // ocp->set_state_guess(guess, "norm_fiber_length_r",
    //                      RowVectorXd::LinSpaced(N, 1.1, 0.9));
    // ocp->set_control_guess(guess, "excitation_r",
    //                        RowVectorXd::LinSpaced(N, 1, 0));
    // ocp->set_control_guess(guess, "norm_fiber_velocity_r",
    //                        RowVectorXd::Constant(N, -0.03));
    // Initializing with a previous solution reduces the number of iterations
    // from 32 to 20.
    tropter::Iterate guess(
            "testTugOfWarDeGrooteFregly2016_INDYGO_initial_guess.csv");
    tropter::Solution ocp_solution = dircol.solve(guess);
    //tropter::Solution ocp_solution = dircol.solve();
    dircol.print_constraint_values(ocp_solution);

    std::string trajectoryFile =
            "testTugOfWarDeGrooteFregly2016_INDYGO_trajectory.csv";
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

    // Create a table containing only the angle and speed of the pendulum.
    TimeSeriesTable ocpSolution(trajFileWithHeader);
    TimeSeriesTable kinematics;
    kinematics.setColumnLabels({"/joint/position/value",
                                "/joint/position/speed"});
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
    TimeSeriesTable actualInvDyn;
    actualInvDyn.setColumnLabels({"inverse_dynamics"});
    auto ocpd = std::make_shared<TugOfWarDynamic<double>>(model);
    for (Eigen::Index iTime = 0; iTime < ocp_solution.time.size(); ++iTime) {
        auto netForce = ocpd->calcNetForce(ocp_solution.states.col(iTime));
        actualInvDyn.appendRow(ocp_solution.time(iTime),
                               SimTK::RowVector(1, netForce));
    }
    CSVFileAdapter::write(actualInvDyn,
                          "DEBUG_testTugOfWar_INDYGO_actualInvDyn.csv");
    return {ocpSolution, kinematics};
}

// Reproduce the trajectory (generated without muscle dynamics) using static
// optimization.
void test2Muscles1DOFGSO(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data,
        const Model& model) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    // Create the GlobalStaticOptimization.
    // ------------------------------------------
    GlobalStaticOptimization gso;
    gso.setModel(model);
    gso.setKinematicsData(kinematics);
    gso.set_lowpass_cutoff_frequency_for_joint_moments(50);
    double reserveOptimalForce = 0.001;
    gso.set_create_reserve_actuators(reserveOptimalForce);
    GlobalStaticOptimization::Solution solution = gso.solve();
    solution.write("testTugOfWarDeGrooteFregly2016_GSO");

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------
    rootMeanSquare(solution.activation, "/left",
                   ocpSolution,         "activation_l",
                   0.005);
    rootMeanSquare(solution.activation, "/right",
                   ocpSolution,         "activation_r",
                   0.01);
    auto reserveForceRMS = reserveOptimalForce *
            solution.other_controls.getDependentColumnAtIndex(0).normRMS();
    SimTK_TEST(reserveForceRMS < 0.0001);
}

// Reproduce the trajectory (generated with muscle dynamics) using the
// INDYGO.
void test2Muscles1DOFINDYGO(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data,
        const Model& model) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    // Create the INDYGO.
    // ----------------------------------
    INDYGO mrs;
    mrs.setModel(model);
    mrs.setKinematicsData(kinematics);
    // Without filtering, the moments have high frequency content,
    // probably related to unfiltered generalized coordinates and getting
    // accelerations from a spline fit.
    mrs.set_lowpass_cutoff_frequency_for_joint_moments(30);
    const double reserveOptimalForce = 0.01;
    mrs.set_create_reserve_actuators(reserveOptimalForce);
    // We constrain initial INDYGO activation to 0 because otherwise
    // activation_l incorrectly starts at ~0.45 (no penalty for large initial
    // activation). We tried setting initial fiber velocity to 0 instead, and
    // this also solved the problem but only if using the *exact* inverse
    // dynamics generalized forces (which are not available in practice).
    // UPDATE: Now that the INDYGO cost includes activations, activation_l
    // starts at nearly 0 but activation_r starts at 0.06, so it is still
    // helpful to add this initial activation constraint, but it's not so
    // necessary anymore.
    mrs.set_zero_initial_activation(true);
    INDYGO::Solution solution = mrs.solve();
    solution.write("testTugOfWarDeGrooteFregly2016_INDYGO");

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------

    compare(solution.activation, "/left",
            ocpSolution,         "activation_l",
            0.05);
    compare(solution.activation, "/right",
            ocpSolution,         "activation_r",
            0.05);

    compare(solution.norm_fiber_length, "/left",
            ocpSolution,                "norm_fiber_length_l",
            0.01);
    compare(solution.norm_fiber_length, "/right",
            ocpSolution,                "norm_fiber_length_r",
            0.01);

    // We use a weaker check for the controls; they don't match as well.
    // excitation_l does not match well at the end of the motion; should
    // go to 0 but ends at 0.15 (b/c of error in inverse dynamics generalized
    // forces introduced by filtering, etc.)
    rootMeanSquare(solution.excitation, "/left",
                   ocpSolution,         "excitation_l",
                   0.02);
    rootMeanSquare(solution.excitation, "/right",
                   ocpSolution,         "excitation_r",
                   0.01);

    rootMeanSquare(solution.norm_fiber_velocity, "/left",
                   ocpSolution,                  "norm_fiber_velocity_l",
                   0.005);
    rootMeanSquare(solution.norm_fiber_velocity, "/right",
                   ocpSolution,                  "norm_fiber_velocity_r",
                   0.03);

    // The reserve force at t = 0 is large to account for an incorrect
    // inverse dynamics moment caused by filtering.
    // But after that, the reserve forces should be small.
    const int numRows = (int)solution.other_controls.getNumRows();
    auto reserveForceRMS = reserveOptimalForce *
            solution.other_controls.getDependentColumnAtIndex(0)
                    .updBlock(1, 0, numRows - 1, 1).normRMS();
    SimTK_TEST(reserveForceRMS < 0.001);
}

int main() {
    SimTK_START_TEST("testTugOfWarDeGrooteFregly2016");
        Model model = buildTugOfWarModel();
        model.finalizeFromProperties();
        {
            auto gsoData =
                    solveForTrajectory_GSO(model);
            SimTK_SUBTEST2(test2Muscles1DOFGSO, gsoData, model);
        }
        {
            auto mrsData = solveForTrajectory_INDYGO(model);
            SimTK_SUBTEST2(test2Muscles1DOFINDYGO, mrsData, model);
        }
    SimTK_END_TEST();
}
