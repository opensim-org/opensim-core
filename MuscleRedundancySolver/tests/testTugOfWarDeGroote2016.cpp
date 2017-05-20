#include <OpenSim/OpenSim.h>
#include <MuscleRedundancySolver.h>
#include <GlobalStaticOptimizationSolver.h>
#include <DeGroote2016Muscle.h>
#include <mesh.h>

#include "testing.h"

using namespace OpenSim;

// The objective of this test is to ensure that MRS functions properly with
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
class DeGroote2016MuscleTugOfWarMinEffortStatic
        : public mesh::OptimalControlProblemNamed<T> {
public:
    const double d = DISTANCE;
    double mass = -1;
    int m_i_position = -1;
    int m_i_speed = -1;
    int m_i_activation_l = -1;
    int m_i_activation_r = -1;
    DeGroote2016Muscle<T> m_muscleL;
    DeGroote2016Muscle<T> m_muscleR;

    DeGroote2016MuscleTugOfWarMinEffortStatic(const Model& model) :
            mesh::OptimalControlProblemNamed<T>("tug_of_war_min_effort") {
        this->set_time(0, 0.5);
        m_i_position =
                this->add_state("position", {-0.02, 0.02}, -0.015, 0.015);
        m_i_speed = this->add_state("speed", {-5, 15}, 0, 0);
        m_i_activation_l = this->add_control("activation_l", {0, 1});
        m_i_activation_r = this->add_control("activation_r", {0, 1});
        mass = dynamic_cast<const Body&>(model.getComponent("body")).get_mass();
        {
            const auto& osimMuscleL =
                    dynamic_cast<const Muscle&>(model.getComponent("left"));
            m_muscleL = DeGroote2016Muscle<T>(
                    osimMuscleL.get_max_isometric_force(),
                    osimMuscleL.get_optimal_fiber_length(),
                    osimMuscleL.get_tendon_slack_length(),
                    osimMuscleL.get_pennation_angle_at_optimal(),
                    osimMuscleL.get_max_contraction_velocity());
        }
        {
            const auto& osimMuscleR =
                    dynamic_cast<const Muscle&>(model.getComponent("right"));
            m_muscleR = DeGroote2016Muscle<T>(
                    osimMuscleR.get_max_isometric_force(),
                    osimMuscleR.get_optimal_fiber_length(),
                    osimMuscleR.get_tendon_slack_length(),
                    osimMuscleR.get_pennation_angle_at_optimal(),
                    osimMuscleR.get_max_contraction_velocity());
        }
    }
    void dynamics(const mesh::VectorX<T>& states,
                  const mesh::VectorX<T>& controls,
                  Eigen::Ref<mesh::VectorX<T>> derivatives) const override {
        // Unpack variables.
        // -----------------
        const T& speed = states[m_i_speed];

        // Multibody kinematics.
        // ---------------------
        derivatives[m_i_position] = speed;

        // Multibody dynamics.
        // -------------------
        const T netForce = calcNetForce(states, controls);
        derivatives[m_i_speed] = netForce / mass;
    }
    T calcNetForce(const mesh::VectorX<T>& states,
                   const mesh::VectorX<T>& controls) const {
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
    void integral_cost(const T& /*time*/,
                       const mesh::VectorX<T>& /*states*/,
                       const mesh::VectorX<T>& controls,
                       T& integrand) const override {
        const auto& controlL = controls[m_i_activation_l];
        const auto& controlR = controls[m_i_activation_r];
        integrand = controlL * controlL + controlR * controlR;
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
class DeGroote2016MuscleTugOfWarMinEffortDynamic
        : public mesh::OptimalControlProblemNamed<T> {
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

    DeGroote2016Muscle<T> m_muscleL;
    DeGroote2016Muscle<T> m_muscleR;

    DeGroote2016MuscleTugOfWarMinEffortDynamic(const Model& model) :
            mesh::OptimalControlProblemNamed<T>("tug_of_war_min_effort") {
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
        m_i_fiber_equilibrium_l =
                this->add_path_constraint("fiber_equilibrium_l", 0);
        m_i_fiber_equilibrium_r =
                this->add_path_constraint("fiber_equilibrium_r", 0);

        mass = dynamic_cast<const Body&>(model.getComponent("body")).get_mass();
        {
            const auto& osimMuscleL =
                    dynamic_cast<const Muscle&>(model.getComponent("left"));
            m_muscleL = DeGroote2016Muscle<T>(
                    osimMuscleL.get_max_isometric_force(),
                    osimMuscleL.get_optimal_fiber_length(),
                    osimMuscleL.get_tendon_slack_length(),
                    osimMuscleL.get_pennation_angle_at_optimal(),
                    osimMuscleL.get_max_contraction_velocity());
        }
        {
            const auto& osimMuscleR =
                    dynamic_cast<const Muscle&>(model.getComponent("right"));
            m_muscleR = DeGroote2016Muscle<T>(
                    osimMuscleR.get_max_isometric_force(),
                    osimMuscleR.get_optimal_fiber_length(),
                    osimMuscleR.get_tendon_slack_length(),
                    osimMuscleR.get_pennation_angle_at_optimal(),
                    osimMuscleR.get_max_contraction_velocity());
        }
    }
    void dynamics(const mesh::VectorX<T>& states,
                  const mesh::VectorX<T>& controls,
                  Eigen::Ref<mesh::VectorX<T>> derivatives) const override {
        // Unpack variables.
        // -----------------
        const T& speed = states[m_i_speed];

        // Multibody kinematics.
        // ---------------------
        derivatives[m_i_position] = speed;

        // Multibody dynamics.
        // -------------------
        const T netForce = calcNetForce(states);

        derivatives[m_i_speed] = netForce / mass;

        // Activation dynamics.
        // --------------------
        const T& activationL = states[m_i_activation_l];
        const T& excitationL = controls[m_i_excitation_l];
        m_muscleL.calcActivationDynamics(excitationL, activationL,
                                         derivatives[m_i_activation_l]);
        const T& activationR = states[m_i_activation_r];
        const T& excitationR = controls[m_i_excitation_r];
        m_muscleR.calcActivationDynamics(excitationR, activationR,
                                         derivatives[m_i_activation_r]);

        // Fiber dynamics.
        // ---------------
        const T& normFibVelL = controls[m_i_norm_fiber_velocity_l];
        const T& normFibVelR = controls[m_i_norm_fiber_velocity_r];
        derivatives[m_i_norm_fiber_length_l] =
                m_muscleL.get_max_contraction_velocity() * normFibVelL;
        derivatives[m_i_norm_fiber_length_r] =
                m_muscleR.get_max_contraction_velocity() * normFibVelR;
    }
    T calcNetForce(const mesh::VectorX<T>& states) const {
        const T& position = states[m_i_position];

        T forceL;
        const T& normFibLenL = states[m_i_norm_fiber_length_l];
        m_muscleL.calcTendonForce(d + position, normFibLenL, forceL);
        T forceR;
        const T& normFibLenR = states[m_i_norm_fiber_length_r];
        m_muscleR.calcTendonForce(d - position, normFibLenR, forceR);

        return -forceL + forceR;
    }
    void path_constraints(unsigned /*i_mesh*/,
                          const T& /*time*/,
                          const mesh::VectorX<T>& states,
                          const mesh::VectorX<T>& controls,
                          Eigen::Ref<mesh::VectorX<T>> constraints)
    const override {
        const T& position = states[m_i_position];
        {
            const T& activationL = states[m_i_activation_l];
            const T& normFibLenL = states[m_i_norm_fiber_length_l];
            const T& normFibVelL = controls[m_i_norm_fiber_velocity_l];
            m_muscleL.calcEquilibriumResidual(activationL, d + position,
                    normFibLenL, normFibVelL,
                    constraints[m_i_fiber_equilibrium_l]);
        }
        {
            const T& activationR = states[m_i_activation_r];
            const T& normFibLenR = states[m_i_norm_fiber_length_r];
            const T& normFibVelR = controls[m_i_norm_fiber_velocity_r];
            m_muscleR.calcEquilibriumResidual(activationR, d - position,
                    normFibLenR, normFibVelR,
                    constraints[m_i_fiber_equilibrium_r]);
        }
    }
    void integral_cost(const T& /*time*/,
                       const mesh::VectorX<T>& /*states*/,
                       const mesh::VectorX<T>& controls,
                       T& integrand) const override {
        const auto& controlL = controls[m_i_excitation_l];
        const auto& controlR = controls[m_i_excitation_r];
        integrand = controlL * controlL + controlR * controlR;
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
    return model;
}

template <typename T>
using TugOfWarStatic = DeGroote2016MuscleTugOfWarMinEffortStatic<T>;

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectory_GSO(const Model& model) {

    // Solve a trajectory optimization problem.
    // ----------------------------------------

    auto ocp = std::make_shared<TugOfWarStatic<adouble>>(model);
    ocp->print_description();
    const int N = 100;
    mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt", N);
    mesh::OptimalControlSolution ocp_solution = dircol.solve();
    std::string trajectoryFile =
            "testTugOfWarDeGroote2016_GSO_trajectory.csv";
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

    // Create a table containing only the angle and speed of the pendulum.
    TimeSeriesTable ocpSolution = CSVFileAdapter::read(trajFileWithHeader);
    TimeSeriesTable kinematics;
    kinematics.setColumnLabels({"joint/position/value",
                                "joint/position/speed"});
    const auto& position = ocpSolution.getDependentColumn("position");
    const auto& speed = ocpSolution.getDependentColumn("speed");
    for (size_t iRow = 0; iRow < ocpSolution.getNumRows(); ++iRow) {
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
using TugOfWarDynamic = DeGroote2016MuscleTugOfWarMinEffortDynamic<T>;

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectory_MRS(const Model& model) {
    // Solve a trajectory optimization problem.
    // ----------------------------------------
    auto ocp = std::make_shared<TugOfWarDynamic<adouble>>(model);
    ocp->print_description();
    const int N = 100;
    mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt", N);
    // Create an initial guess.
    // This initial guess reduces the number of iterations from 52 to 32.
    // using Eigen::RowVectorXd;
    // mesh::OptimalControlIterate guess;
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
    mesh::OptimalControlIterate guess(
            "testTugOfWarDeGroote2016_MRS_initial_guess.csv");
    mesh::OptimalControlSolution ocp_solution = dircol.solve(guess);
    //mesh::OptimalControlSolution ocp_solution = dircol.solve();
    dircol.print_constraint_values(ocp_solution);

    std::string trajectoryFile =
            "testTugOfWarDeGroote2016_MRS_trajectory.csv";
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

    // Create a table containing only the angle and speed of the pendulum.
    TimeSeriesTable ocpSolution = CSVFileAdapter::read(trajFileWithHeader);
    TimeSeriesTable kinematics;
    kinematics.setColumnLabels({"joint/position/value",
                                "joint/position/speed"});
    const auto& position = ocpSolution.getDependentColumn("position");
    const auto& speed = ocpSolution.getDependentColumn("speed");
    for (size_t iRow = 0; iRow < ocpSolution.getNumRows(); ++iRow) {
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
                          "DEBUG_testTugOfWar_MRS_actualInvDyn.csv");
    return {ocpSolution, kinematics};
}

// Reproduce the trajectory (generated without muscle dynamics) using static
// optimization.
void test2Muscles1DOFGlobalStaticOptimizationSolver(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data,
        const Model& model) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    // Create the GlobalStaticOptimizationSolver.
    // ------------------------------------------
    GlobalStaticOptimizationSolver gso;
    gso.setModel(model);
    gso.setKinematicsData(kinematics);
    gso.set_lowpass_cutoff_frequency_for_joint_moments(50);
    double reserveOptimalForce = 0.001;
    gso.set_create_reserve_actuators(reserveOptimalForce);
    GlobalStaticOptimizationSolver::Solution solution = gso.solve();
    solution.write("testTugOfWarDeGroote2016_GSO");

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------
    rootMeanSquare(solution.activation, "/tug_of_war/left",
                   ocpSolution,         "activation_l",
                   0.005);
    rootMeanSquare(solution.activation, "/tug_of_war/right",
                   ocpSolution,         "activation_r",
                   0.01);
    auto reserveForceRMS = reserveOptimalForce *
            solution.other_controls.getDependentColumnAtIndex(0).normRMS();
    SimTK_TEST(reserveForceRMS < 0.0001);
}

// Reproduce the trajectory (generated with muscle dynamics) using the
// MuscleRedundancySolver.
void test2Muscles1DOFMuscleRedundancySolver(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data,
        const Model& model) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    // Create the MuscleRedundancySolver.
    // ----------------------------------
    MuscleRedundancySolver mrs;
    mrs.setModel(model);
    mrs.setKinematicsData(kinematics);
    // Without filtering, the moments have high frequency content,
    // probably related to unfiltered generalized coordinates and getting
    // accelerations from a spline fit.
    mrs.set_lowpass_cutoff_frequency_for_joint_moments(30);
    const double reserveOptimalForce = 0.01;
    mrs.set_create_reserve_actuators(reserveOptimalForce);
    // We constrain initial MRS activation to 0 because otherwise activation_l
    // incorrectly starts at ~0.45 (no penalty for large initial activation).
    // We tried setting initial fiber velocity to 0 instead, and this also
    // solved the problem but only if using the *exact* inverse dynamics
    // generalized forces (which are not available in practice).
    mrs.set_zero_initial_activation(true);
    MuscleRedundancySolver::Solution solution = mrs.solve();
    solution.write("testTugOfWarDeGroote2016_MRS");

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------

    compare(solution.activation, "/tug_of_war/left",
            ocpSolution,         "activation_l",
            0.05);
    compare(solution.activation, "/tug_of_war/right",
            ocpSolution,         "activation_r",
            0.05);

    compare(solution.norm_fiber_length, "/tug_of_war/left",
            ocpSolution,                "norm_fiber_length_l",
            0.01);
    compare(solution.norm_fiber_length, "/tug_of_war/right",
            ocpSolution,                "norm_fiber_length_r",
            0.01);

    // We use a weaker check for the controls; they don't match as well.
    // excitation_l does not match well at the end of the motion; should
    // go to 0 but ends at 0.15 (b/c of error in inverse dynamics generalized
    // forces introduced by filtering, etc.)
    rootMeanSquare(solution.excitation, "/tug_of_war/left",
                   ocpSolution,         "excitation_l",
                   0.02);
    rootMeanSquare(solution.excitation, "/tug_of_war/right",
                   ocpSolution,         "excitation_r",
                   0.01);

    rootMeanSquare(solution.norm_fiber_velocity, "/tug_of_war/left",
                   ocpSolution,                  "norm_fiber_velocity_l",
                   0.005);
    rootMeanSquare(solution.norm_fiber_velocity, "/tug_of_war/right",
                   ocpSolution,                  "norm_fiber_velocity_r",
                   0.03);

    // The reserve force at t = 0 is large to account for an incorrect
    // inverse dynamics moment caused by filtering.
    // But after that, the reserve forces should be small.
    const auto numRows = solution.other_controls.getNumRows();
    auto reserveForceRMS = reserveOptimalForce *
            solution.other_controls.getDependentColumnAtIndex(0)
                    .updBlock(1, 0, numRows - 1, 1).normRMS();
    SimTK_TEST(reserveForceRMS < 0.001);
}

int main() {
    SimTK_START_TEST("testTugOfWarDeGroote2016");
        Model model = buildTugOfWarModel();
        model.finalizeFromProperties();
        {
            auto gsoData =
                    solveForTrajectory_GSO(model);
            SimTK_SUBTEST2(test2Muscles1DOFGlobalStaticOptimizationSolver,
                           gsoData, model);
        }
        {
            auto mrsData = solveForTrajectory_MRS(model);
            SimTK_SUBTEST2(test2Muscles1DOFMuscleRedundancySolver, mrsData,
                           model);
        }
    SimTK_END_TEST();
}
