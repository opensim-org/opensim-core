/* -------------------------------------------------------------------------- *
 * OpenSim Moco: test2Muscles2DOFsDeGrooteFregly2016.cpp                      *
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
#include <Moco/InverseMuscleSolver/InverseMuscleSolverMotionData.h>

#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>

#include <tropter/tropter.h>

using namespace OpenSim;

// The objective of this test is to ensure that INDYGO functions properly with
// multiple muscles and multiple degrees of freedom.

// The horizontal distance between the muscle origins and the origin of the
// global coordinate system (0, 0).
const double WIDTH = 0.2;
const double ACCEL_GRAVITY = 9.81;

/// Move a point mass from a fixed starting state to a fixed end
/// position and velocity, in fixed time, with minimum effort. The point mass
/// has 2 DOFs (x and y translation).
///
///                            |< d >|< d >|
///                    ----------------------
///                             \         /
///                              .       .
///                   left muscle \     / right muscle
///                                .   .
///                                 \ /
///                                  O mass
///
/// Here's a sketch of the problem we solve (rigid tendon, no activ.dynamics)
/// @verbatim
///   minimize   int_t (aL^2 + aR^2) dt
///   subject to xdot = vx                                        kinematics
///              ydot = vy                                        kinematics
///              vxdot = 1/m (-f_tL (d+x)/lmtL + f_tR (d-x)/lmtR) dynamics
///              vydot = 1/m (-f_tL (-y)/lmtL + f_tR (-y)/lmtR)   dynamics
///              f_tL = (aL f_l(lmL) f_v(vmL) + f_p(lmL)) cos(alphaL)
///              f_tR = (aR f_l(lmR) f_v(vmR) + f_p(lmR)) cos(alphaR)
///              x(0) = -0.03
///              y(0) = -d
///              vx(0) = 0
///              vy(0) = 0
///              aL(0) = 0
///              aR(0) = 0
///              x(0.5) = +0.03
///              y(0.5) = -d + 0.05
///              vx(0.5) = 0
///              vy(0.5) = 0
/// @endverbatim
template <typename T>
class OCPStatic : public tropter::Problem<T> {
public:
    const double d = WIDTH;
    double mass = -1;
    int m_i_x = -1;
    int m_i_y = -1;
    int m_i_vx = -1;
    int m_i_vy = -1;
    int m_i_activation_l = -1;
    int m_i_activation_r = -1;
    DeGrooteFregly2016MuscleStandalone<T> m_muscleL;
    DeGrooteFregly2016MuscleStandalone<T> m_muscleR;
    struct NetForce {
        T x;
        T y;
    };

    OCPStatic(const Model& model) :
            tropter::Problem<T>("2musc2dofstatic") {
        this->set_time(0, 0.2);
        m_i_x = this->add_state("x", {-0.03, 0.03}, -0.03, 0.03);
        m_i_y = this->add_state("y", {-2 * d, 0}, -d, -d + 0.05);
        m_i_vx = this->add_state("vx", {-15, 15}, 0, 0);
        m_i_vy = this->add_state("vy", {-15, 15}, 0, 0);
        m_i_activation_l = this->add_control("activation_l", {0, 1});
        m_i_activation_r = this->add_control("activation_r", {0, 1});
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
        this->add_cost("effort", 1);
    }
    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {
        // Unpack variables.
        // -----------------
        const T& vx = in.states[m_i_vx];
        const T& vy = in.states[m_i_vy];

        // Multibody kinematics.
        // ---------------------
        out.dynamics[m_i_x] = vx;
        out.dynamics[m_i_y] = vy;

        // Multibody dynamics.
        // -------------------
        const auto netForce = calcNetForce(in.states, in.controls);
        out.dynamics[m_i_vx] = netForce.x / mass;
        out.dynamics[m_i_vy] = netForce.y / mass - ACCEL_GRAVITY;
    }
    NetForce calcNetForce(
            const Eigen::Ref<const tropter::VectorX<T>>& states,
            const Eigen::Ref<const tropter::VectorX<T>>& controls) const {
        const T& x = states[m_i_x];
        const T& y = states[m_i_y];
        const T& vx = states[m_i_vx];
        const T& vy = states[m_i_vy];

        const T& activationL = controls[m_i_activation_l];
        const T musTenLenL = sqrt(pow(d + x, 2) + pow(y, 2));
        const T musTenVelL = ((d + x) * vx + y * vy) / musTenLenL;
        const T tensionL = m_muscleL.calcRigidTendonFiberForceAlongTendon(
                activationL, musTenLenL, musTenVelL);

        const T& activationR = controls[m_i_activation_r];
        const T musTenLenR = sqrt(pow(d - x, 2) + pow(y, 2));
        const T musTenVelR = (-(d - x) * vx + y * vy) / musTenLenR;
        const T tensionR = m_muscleR.calcRigidTendonFiberForceAlongTendon(
                activationR, musTenLenR, musTenVelR);

        const T netForceX = -tensionL * (d + x) / musTenLenL
                            +tensionR * (d - x) / musTenLenR;
        const T netForceY = +tensionL * (-y) / musTenLenL
                            +tensionR * (-y) / musTenLenR;
        return {netForceX,  netForceY};
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

/// Move a point mass from a fixed starting state to a fixed end
/// position and velocity, in fixed time, with minimum effort. The point mass
/// has 2 DOFs (x and y translation).
///
///                            |< d >|< d >|
///                    ----------------------
///                             \         /
///                              .       .
///                   left muscle \     / right muscle
///                                .   .
///                                 \ /
///                                  O mass
///
/// Here's a sketch of the problem we solve, with activation and fiber dynamics.
/// @verbatim
///   minimize   int_t (aL^2 + aR^2) dt
///   subject to xdot = vx                                        kinematics
///              ydot = vy                                        kinematics
///              vxdot = 1/m (-f_tL (d+x)/lmtL + f_tR (d-x)/lmtR) dynamics
///              vydot = 1/m (-f_tL (-y)/lmtL + f_tR (-y)/lmtR)   dynamics
///              aLdot = f_a(eL, aL)       activation dynamics
///              aRdot = f_a(eR, aR)
///              lmLdot = vmLdot           fiber dynamics
///              lmRdot = vmRdot
///(for L and R) (a f_l(lm) f_v(vm) + f_p(lm)) cos(alpha) = f_t(lt) equilibrium
///              x(0) = -0.03
///              y(0) = -d
///              vx(0) = 0
///              vy(0) = 0
///              aL(0) = 0
///              aR(0) = 0
///              vmL(0) = 0
///              vmR(0) = 0
///              x(0.5) = +0.03
///              y(0.5) = -d + 0.05
///              vx(0.5) = 0
///              vy(0.5) = 0
/// @endverbatim
template <typename T>
class OCPDynamic : public tropter::Problem<T> {
public:
    const double d = WIDTH;
    double mass = -1;
    int m_i_x = -1;
    int m_i_y = -1;
    int m_i_vx = -1;
    int m_i_vy = -1;
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
    struct NetForce {
        T x;
        T y;
    };
    OCPDynamic(const Model& model) :
            tropter::Problem<T>("2musc2dofdynamic") {
        this->set_time(0, 0.5);
        m_i_x = this->add_state("x", {-0.03, 0.03}, -0.03, 0.03);
        m_i_y = this->add_state("y", {-2 * d, 0}, -d, -d + 0.05);
        m_i_vx = this->add_state("vx", {-15, 15}, 0, 0);
        m_i_vy = this->add_state("vy", {-15, 15}, 0, 0);
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
        // Unpack variables.
        // -----------------
        const T& vx = in.states[m_i_vx];
        const T& vy = in.states[m_i_vy];

        // Multibody kinematics.
        // ---------------------
        out.dynamics[m_i_x] = vx;
        out.dynamics[m_i_y] = vy;

        // Multibody dynamics.
        // -------------------
        const auto netForce = calcNetForce(in.states);
        out.dynamics[m_i_vx] = netForce.x / mass;
        out.dynamics[m_i_vy] = netForce.y / mass - ACCEL_GRAVITY;

        // Activation dynamics.
        // --------------------
        const T& activationL = in.states[m_i_activation_l];
        const T& excitationL = in.controls[m_i_excitation_l];
        m_muscleL.calcActivationDynamics(excitationL, activationL,
                                         out.dynamics[m_i_activation_l]);
        const T& activationR = in.states[m_i_activation_r];
        const T& excitationR = in.controls[m_i_excitation_r];
        m_muscleR.calcActivationDynamics(excitationR, activationR,
                                         out.dynamics[m_i_activation_r]);

        // Fiber dynamics.
        // ---------------
        const T& normFibVelL = in.controls[m_i_norm_fiber_velocity_l];
        const T& normFibVelR = in.controls[m_i_norm_fiber_velocity_r];
        out.dynamics[m_i_norm_fiber_length_l] =
                m_muscleL.get_max_contraction_velocity() * normFibVelL;
        out.dynamics[m_i_norm_fiber_length_r] =
                m_muscleR.get_max_contraction_velocity() * normFibVelR;

        // Path constraints.
        // =================
        if (out.path.size() != 0) {
            const T& x = in.states[m_i_x];
            const T& y = in.states[m_i_y];
            {
                const T& activationL = in.states[m_i_activation_l];
                const T& normFibLenL = in.states[m_i_norm_fiber_length_l];
                const T& normFibVelL = in.controls[m_i_norm_fiber_velocity_l];
                const T musTenLenL = sqrt(pow(d + x, 2) + pow(y, 2));
                m_muscleL.calcEquilibriumResidual(activationL, musTenLenL,
                        normFibLenL, normFibVelL,
                        out.path[m_i_fiber_equilibrium_l]);
            }
            {
                const T& activationR = in.states[m_i_activation_r];
                const T& normFibLenR = in.states[m_i_norm_fiber_length_r];
                const T& normFibVelR = in.controls[m_i_norm_fiber_velocity_r];
                const T musTenLenR = sqrt(pow(d - x, 2) + pow(y, 2));
                m_muscleR.calcEquilibriumResidual(activationR, musTenLenR,
                        normFibLenR, normFibVelR,
                        out.path[m_i_fiber_equilibrium_r]);
            }
        }
    }
    NetForce calcNetForce(const tropter::VectorX<T>& states) const {
        const T& x = states[m_i_x];
        const T& y = states[m_i_y];

        T tensionL;
        const T musTenLenL = sqrt(pow(d + x, 2) + pow(y, 2));
        const T& normFibLenL = states[m_i_norm_fiber_length_l];
        m_muscleL.calcTendonForce(musTenLenL, normFibLenL, tensionL);

        T tensionR;
        const T musTenLenR = sqrt(pow(d - x, 2) + pow(y, 2));
        const T& normFibLenR = states[m_i_norm_fiber_length_r];
        m_muscleR.calcTendonForce(musTenLenR, normFibLenR, tensionR);

        const T netForceX = -tensionL * (d + x) / musTenLenL
                            +tensionR * (d - x) / musTenLenR;
        const T netForceY = +tensionL * (-y) / musTenLenL
                            +tensionR * (-y) / musTenLenR;
        return {netForceX,  netForceY};
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

OpenSim::Model buildModel() {
    using SimTK::Vec3;

    Model model;
    // model.setUseVisualizer(true);
    model.setName("block2musc2dof");
    model.set_gravity(Vec3(0, -ACCEL_GRAVITY, 0));

    // Massless intermediate body.
    auto* intermed = new Body("intermed", 0, Vec3(0), SimTK::Inertia(0));
    model.addComponent(intermed);
    auto* body = new Body("body", 1, Vec3(0), SimTK::Inertia(1));
    model.addComponent(body);

    // TODO GSO and INDYGO do not support locked coordinates yet.
    // Allow translation along x and y; disable rotation about z.
    // auto* joint = new PlanarJoint();
    // joint->setName("joint");
    // joint->connectSocket_parent_frame(model.getGround());
    // joint->connectSocket_child_frame(*body);
    // auto& coordTX = joint->updCoordinate(PlanarJoint::Coord::TranslationX);
    // coordTX.setName("tx");
    // auto& coordTY = joint->updCoordinate(PlanarJoint::Coord::TranslationY);
    // coordTY.setName("ty");
    // auto& coordRZ = joint->updCoordinate(PlanarJoint::Coord::RotationZ);
    // coordRZ.setName("rz");
    // coordRZ.setDefaultLocked(true);
    // model.addComponent(joint);
    auto* jointX = new SliderJoint();
    jointX->setName("tx");
    jointX->connectSocket_parent_frame(model.getGround());
    jointX->connectSocket_child_frame(*intermed);
    auto& coordX = jointX->updCoordinate(SliderJoint::Coord::TranslationX);
    coordX.setName("tx");
    model.addComponent(jointX);

    // The joint's x axis must point in the global "+y" direction.
    auto* jointY = new SliderJoint("ty",
            *intermed, Vec3(0), Vec3(0, 0, 0.5 * SimTK::Pi),
            *body, Vec3(0), Vec3(0, 0, .5 * SimTK::Pi));
    auto& coordY = jointY->updCoordinate(SliderJoint::Coord::TranslationX);
    coordY.setName("ty");
    model.addComponent(jointY);

    {
        auto* actuL = new Millard2012EquilibriumMuscle();
        actuL->setName("left");
        actuL->set_max_isometric_force(40);
        actuL->set_optimal_fiber_length(.20);
        actuL->set_tendon_slack_length(0.10);
        actuL->set_pennation_angle_at_optimal(0.0);
        actuL->addNewPathPoint("origin", model.updGround(),
                               Vec3(-WIDTH, 0, 0));
        actuL->addNewPathPoint("insertion", *body, Vec3(0));
        model.addComponent(actuL);
    }
    {
        auto* actuR = new Millard2012EquilibriumMuscle();
        actuR->setName("right");
        actuR->set_max_isometric_force(40);
        actuR->set_optimal_fiber_length(.21);
        actuR->set_tendon_slack_length(0.09);
        actuR->set_pennation_angle_at_optimal(0.0);
        actuR->addNewPathPoint("origin", model.updGround(),
                               Vec3(+WIDTH, 0, 0));
        actuR->addNewPathPoint("insertion", *body, Vec3(0));
        model.addComponent(actuR);
    }

    model.finalizeConnections();

    // For use in "filebased" tests.
    model.print("test2Muscles2DOFsDeGrooteFregly2016.osim");
    // SimTK::State s = model.initSystem();
    // model.equilibrateMuscles(s);
    // model.updMatterSubsystem().setShowDefaultGeometry(true);
    // model.updVisualizer().updSimbodyVisualizer().setBackgroundType(
    //         SimTK::Visualizer::GroundAndSky);
    // model.getVisualizer().show(s);
    // std::cin.get();
    // Manager manager(model);
    // manager.integrate(s, 1.0);
    // std::cin.get();
    return model;
}

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectory_GSO(const Model& model) {

    // Solve a trajectory optimization problem.
    auto ocp = std::make_shared<OCPStatic<adouble>>(model);
    ocp->print_description();
    const int N = 100;
    tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt", N);
    tropter::Solution ocp_solution = dircol.solve();
    std::string trajectoryFile =
            "test2Muscles2DOFsDeGrooteFregly2016_GSO_trajectory.csv";
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
    kinematics.setColumnLabels(
            {"/tx/tx/value", "/tx/tx/speed", "/ty/ty/value", "/ty/ty/speed"});
    const auto& x = ocpSolution.getDependentColumn("x");
    const auto& vx = ocpSolution.getDependentColumn("vx");
    const auto& y = ocpSolution.getDependentColumn("y");
    const auto& vy = ocpSolution.getDependentColumn("vy");
    SimTK::RowVector row(4);
    for (int iRow = 0; iRow < (int)ocpSolution.getNumRows(); ++iRow) {
        row[0] = x[iRow];
        row[1] = vx[iRow];
        row[2] = y[iRow];
        row[3] = vy[iRow];
        kinematics.appendRow(ocpSolution.getIndependentColumn()[iRow], row);
    }
    // For use in the "filebased" test.
    CSVFileAdapter::write(kinematics,
            "test2Muscles2DOFsDeGrooteFregly2016_GSO_kinematics.csv");

    // Compute actual inverse dynamics moment, for debugging.
    // ------------------------------------------------------
    TimeSeriesTable actualInvDyn;
    actualInvDyn.setColumnLabels({"x", "y"});
    auto ocpd = std::make_shared<OCPStatic<double>>(model);
    for (Eigen::Index iTime = 0; iTime < ocp_solution.time.size(); ++iTime) {
        auto netForce = ocpd->calcNetForce(ocp_solution.states.col(iTime),
                                           ocp_solution.controls.col(iTime));
        SimTK::RowVector row(2);
        row[0] = netForce.x;
        row[1] = netForce.y;
        actualInvDyn.appendRow(ocp_solution.time(iTime), row);
    }
    CSVFileAdapter::write(actualInvDyn,
                          "DEBUG_test2Muscles2DOFs_GSO_actualInvDyn.csv");

    return {ocpSolution, kinematics};
}

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectory_INDYGO(const Model& model) {
    // Solve a trajectory optimization problem.
    // ----------------------------------------
    auto ocp = std::make_shared<OCPDynamic<adouble>>(model);
    ocp->print_description();
    const int N = 100;
    tropter::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt", N);
    // The quasi-Newton mode gives a big speedup for this problem.
    dircol.get_opt_solver().set_hessian_approximation("limited-memory");

    tropter::Iterate guess(
            "test2Muscles2DOFsDeGrooteFregly2016_INDYGO_initial_guess.csv");
    tropter::Solution ocp_solution = dircol.solve(guess);
    //tropter::Solution ocp_solution = dircol.solve();
    dircol.print_constraint_values(ocp_solution);

    std::string trajectoryFile =
            "test2Muscles2DOFsDeGrooteFregly2016_INDYGO_trajectory.csv";
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
    kinematics.setColumnLabels(
            {"/tx/tx/value", "/tx/tx/speed", "/ty/ty/value", "/ty/ty/speed"});
    const auto& x = ocpSolution.getDependentColumn("x");
    const auto& vx = ocpSolution.getDependentColumn("vx");
    const auto& y = ocpSolution.getDependentColumn("y");
    const auto& vy = ocpSolution.getDependentColumn("vy");
    SimTK::RowVector row(4);
    for (int iRow = 0; iRow < (int)ocpSolution.getNumRows(); ++iRow) {
        row[0] = x[iRow];
        row[1] = vx[iRow];
        row[2] = y[iRow];
        row[3] = vy[iRow];
        kinematics.appendRow(ocpSolution.getIndependentColumn()[iRow], row);
    }
    // For use in the "filebased" test.
    CSVFileAdapter::write(kinematics,
            "test2Muscles2DOFsDeGrooteFregly2016_INDYGO_kinematics.csv");

    // Compute actual inverse dynamics moment, for debugging.
    // ------------------------------------------------------
    TimeSeriesTable actualInvDyn;
    actualInvDyn.setColumnLabels({"x", "y"});
    auto ocpd = std::make_shared<OCPDynamic<double>>(model);
    for (Eigen::Index iTime = 0; iTime < ocp_solution.time.size(); ++iTime) {
        auto netForce = ocpd->calcNetForce(ocp_solution.states.col(iTime));
        SimTK::RowVector row(2);
        row[0] = netForce.x;
        row[1] = netForce.y;
        actualInvDyn.appendRow(ocp_solution.time(iTime), row);
    }
    CSVFileAdapter::write(actualInvDyn,
                          "DEBUG_test2Muscles2DOFs_INDYGO_actualInvDyn.csv");
    return {ocpSolution, kinematics};
}

void compareSolution_GSO(const GlobalStaticOptimization::Solution& actual,
                         const TimeSeriesTable& expected,
                         const double& reserveOptimalForce) {
    compare(actual.activation, "/left",
            expected,          "activation_l",
            0.02);
    compare(actual.activation, "/right",
            expected,          "activation_r",
            0.05);
    auto reserveForceXRMS = reserveOptimalForce *
            actual.other_controls.getDependentColumnAtIndex(0).normRMS();
    SimTK_TEST(reserveForceXRMS < 0.05);
    auto reserveForceYRMS = reserveOptimalForce *
            actual.other_controls.getDependentColumnAtIndex(1).normRMS();
    SimTK_TEST(reserveForceYRMS < 0.07);
}

void test2Muscles2DOFs_GSO(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data,
        const Model& model) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    GlobalStaticOptimization gso;
    gso.setModel(model);
    gso.setKinematicsData(kinematics);
    gso.set_lowpass_cutoff_frequency_for_joint_moments(80);
    double reserveOptimalForce = 0.001;
    gso.set_create_reserve_actuators(reserveOptimalForce);
    gso.set_mesh_point_frequency(500);
    // gso.print("test2Muscles2DOFsDeGrooteFregly2016_GSO_setup.xml");
    GlobalStaticOptimization::Solution solution = gso.solve();
    // solution.write("test2Muscles2DOFsDeGrooteFregly2016_GSO");

    // Compare the solution to the initial trajectory optimization solution.
    compareSolution_GSO(solution, ocpSolution, reserveOptimalForce);
}

// Load all settings from a setup file, and run the same tests as in the test
// above.
void test2Muscles2DOFs_GSO_Filebased(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data) {
    const auto& ocpSolution = data.first;

    GlobalStaticOptimization gso(
            "test2Muscles2DOFsDeGrooteFregly2016_GSO_setup.xml");
    gso.set_mesh_point_frequency(500);
    double reserveOptimalForce = gso.get_create_reserve_actuators();
    GlobalStaticOptimization::Solution solution = gso.solve();

    // Compare the solution to the initial trajectory optimization solution.
    compareSolution_GSO(solution, ocpSolution, reserveOptimalForce);
}

void test2Muscles2DOFs_GSO_GenForces(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data,
        const Model& model) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    // Run inverse dynamics.
    Model modelForID = model;
    modelForID.initSystem();
    std::vector<const Coordinate*> coordsToActuate;
    for (auto& coord : modelForID.getCoordinatesInMultibodyTreeOrder())
        coordsToActuate.push_back(coord.get());
    InverseMuscleSolverMotionData motionData(modelForID, coordsToActuate,
            0, 0.2, kinematics, -1, 80);
    Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(100, 0, 0.2);
    Eigen::MatrixXd netGenForcesEigen;
    motionData.interpolateNetGeneralizedForces(times, netGenForcesEigen);
    // Convert Eigen Matrix to TimeSeriesTable.
    // This constructor expects a row-major matrix layout, but the Eigen matrix
    // is column-major. We can exploit this to transpose the data from
    // "DOFs x time" to "time x DOFs".
    SimTK::Matrix netGenForcesMatrix(
            (int)netGenForcesEigen.cols(), (int)netGenForcesEigen.rows(),
            netGenForcesEigen.data());
    TimeSeriesTable netGenForces;
    netGenForces.setColumnLabels({"/tx/tx", "/ty/ty"});
    for (int iRow = 0; iRow < netGenForcesMatrix.nrow(); ++iRow) {
        netGenForces.appendRow(times[iRow], netGenForcesMatrix.row(iRow));
    }

    GlobalStaticOptimization gso;
    gso.setModel(model);
    gso.setKinematicsData(kinematics);
    gso.setNetGeneralizedForcesData(netGenForces);
    double reserveOptimalForce = 0.001;
    gso.set_create_reserve_actuators(reserveOptimalForce);
    gso.set_mesh_point_frequency(500);
    GlobalStaticOptimization::Solution solution = gso.solve();

    // Compare the solution to the initial trajectory optimization solution.
    compareSolution_GSO(solution, ocpSolution, reserveOptimalForce);
}

void compareSolution_INDYGO(const INDYGO::Solution& actual,
                         const TimeSeriesTable& expected,
                         const double& reserveOptimalForce) {
    compare(actual.activation, "/left",
            expected,          "activation_l",
            0.05);
    compare(actual.activation, "/right",
            expected,          "activation_r",
            0.05);

    compare(actual.norm_fiber_length, "/left",
            expected,                 "norm_fiber_length_l",
            0.01);
    compare(actual.norm_fiber_length, "/right",
            expected,                 "norm_fiber_length_r",
            0.01);

    // We use a weaker check for the controls; they don't match as well.
    // The excitations are fairly noisy across time, and the fiber velocity
    // does not match well at the beginning of the trajectory.
    rootMeanSquare(actual.excitation, "/left",
            expected,                 "excitation_l",
            0.03);
    rootMeanSquare(actual.excitation, "/right",
            expected,                 "excitation_r",
            0.03);

    rootMeanSquare(actual.norm_fiber_velocity, "/left",
            expected,                          "norm_fiber_velocity_l",
            0.02);
    rootMeanSquare(actual.norm_fiber_velocity, "/right",
            expected,                          "norm_fiber_velocity_r",
            0.02);

    auto reserveForceXRMS = reserveOptimalForce *
            actual.other_controls.getDependentColumnAtIndex(0).normRMS();
    SimTK_TEST(reserveForceXRMS < 0.05);
    auto reserveForceYRMS = reserveOptimalForce *
            actual.other_controls.getDependentColumnAtIndex(1).normRMS();
    SimTK_TEST(reserveForceYRMS < 0.15);
}

// Reproduce the trajectory (generated with muscle dynamics) using the
// INDYGO.
void test2Muscles2DOFs_INDYGO(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data,
        const Model& model) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    // Create the INDYGO.
    // ----------------------------------
    INDYGO mrs;
    mrs.setModel(model);
    mrs.setKinematicsData(kinematics);
    mrs.set_lowpass_cutoff_frequency_for_joint_moments(20);
    const double reserveOptimalForce = 0.01;
    mrs.set_create_reserve_actuators(reserveOptimalForce);
    // We constrain initial INDYGO activation to 0 because otherwise activation
    // incorrectly starts at a large value (no penalty for large initial
    // activation).
    mrs.set_zero_initial_activation(true);
    // mrs.print("test2Muscles2DOFsDeGrooteFregly2016_INDYGO_setup.xml");
    INDYGO::Solution solution = mrs.solve();
    // solution.write("test2Muscles2DOFsDeGrooteFregly2016_INDYGO");

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------
    compareSolution_INDYGO(solution, ocpSolution, reserveOptimalForce);
}

// Load INDYGO from an XML file.
void test2Muscles2DOFs_INDYGO_Filebased(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data) {
    const auto& ocpSolution = data.first;

    // Create the INDYGO.
    INDYGO mrs
            ("test2Muscles2DOFsDeGrooteFregly2016_INDYGO_setup.xml");
    double reserveOptimalForce = mrs.get_create_reserve_actuators();
    INDYGO::Solution solution = mrs.solve();

    // Compare the solution to the initial trajectory optimization solution.
    compareSolution_INDYGO(solution, ocpSolution, reserveOptimalForce);
}

// Perform inverse dynamics outside of INDYGO.
void test2Muscles2DOFs_INDYGO_GenForces(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data,
        const Model& model) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    // Run inverse dynamics.
    // ---------------------
    // This constructor performs inverse dynamics:
    Model modelForID = model;
    modelForID.initSystem();
    std::vector<const Coordinate*> coordsToActuate;
    for (auto& coord : modelForID.getCoordinatesInMultibodyTreeOrder())
        coordsToActuate.push_back(coord.get());
    InverseMuscleSolverMotionData motionData(modelForID, coordsToActuate,
            0, 0.5, kinematics, -1, 20);
    Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(100, 0, 0.5);
    Eigen::MatrixXd netGenForcesEigen;
    motionData.interpolateNetGeneralizedForces(times, netGenForcesEigen);
    // Convert Eigen Matrix to TimeSeriesTable.
    // This constructor expects a row-major matrix layout, but the Eigen matrix
    // is column-major. We can exploit this to transpose the data from
    // "DOFs x time" to "time x DOFs".
    SimTK::Matrix netGenForcesMatrix(
            (int)netGenForcesEigen.cols(), (int)netGenForcesEigen.rows(),
            netGenForcesEigen.data());
    TimeSeriesTable netGenForces;
    netGenForces.setColumnLabels({"/tx/tx", "/ty/ty"});
    for (int iRow = 0; iRow < netGenForcesMatrix.nrow(); ++iRow) {
        netGenForces.appendRow(times[iRow], netGenForcesMatrix.row(iRow));
    }

    // Create the INDYGO.
    // ----------------------------------
    INDYGO mrs;
    mrs.setModel(model);
    mrs.setKinematicsData(kinematics);
    const double reserveOptimalForce = 0.01;
    mrs.set_create_reserve_actuators(reserveOptimalForce);
    mrs.set_zero_initial_activation(true);
    mrs.setNetGeneralizedForcesData(netGenForces);
    INDYGO::Solution solution = mrs.solve();

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------
    compareSolution_INDYGO(solution, ocpSolution, reserveOptimalForce);
}

int main() {
    SimTK_START_TEST("test2Muscles2DOFsDeGrooteFregly2016");
        Model model = buildModel();
        model.finalizeFromProperties();
        {
            auto data = solveForTrajectory_GSO(model);
            SimTK_SUBTEST2(test2Muscles2DOFs_GSO, data, model);
            SimTK_SUBTEST1(test2Muscles2DOFs_GSO_Filebased, data);
            SimTK_SUBTEST2(test2Muscles2DOFs_GSO_GenForces, data, model);
        }
        {
            auto data = solveForTrajectory_INDYGO(model);
            SimTK_SUBTEST2(test2Muscles2DOFs_INDYGO, data, model);
            SimTK_SUBTEST1(test2Muscles2DOFs_INDYGO_Filebased, data);
            SimTK_SUBTEST2(test2Muscles2DOFs_INDYGO_GenForces, data, model);
        }
    SimTK_END_TEST();
}
