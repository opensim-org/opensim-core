#include <OpenSim/OpenSim.h>
#include <MuscleRedundancySolver.h>
#include <GlobalStaticOptimizationSolver.h>
#include <DeGroote2016Muscle.h>
#include <mesh.h>

#include "testing.h"

using namespace OpenSim;

// The objective of this test is to ensure that MRS functions properly with
// multiple muscles and multiple degrees of freedom.

// The horizontal distance between the muscle origins and the origin of the
// global coordinate system (0, 0).
const double WIDTH = 0.2;
const double ACCEL_GRAVITY = 9.81;

// TODO document
template <typename T>
class OCPStatic : public mesh::OptimalControlProblemNamed<T> {
public:
    const double d = WIDTH;
    double mass = -1;
    int m_i_x = -1;
    int m_i_y = -1;
    int m_i_vx = -1;
    int m_i_vy = -1;
    int m_i_activation_l = -1;
    int m_i_activation_r = -1;
    DeGroote2016Muscle<T> m_muscleL;
    DeGroote2016Muscle<T> m_muscleR;
    struct NetForce {
        T x;
        T y;
    };

    OCPStatic(const Model& model) :
            mesh::OptimalControlProblemNamed<T>("2musc2dof") {
        this->set_time(0, 0.2);
        m_i_x = this->add_state("x", {-0.02, 0.02}, -0.02, 0.02);
        m_i_y = this->add_state("y", {-2 * d, 0}, -d, -d + 0.05);
        m_i_vx = this->add_state("vx", {-15, 15}, 0, 0);
        m_i_vy = this->add_state("vy", {-15, 15}, 0, 0);
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
        const T& vx = states[m_i_vx];
        const T& vy = states[m_i_vy];

        // Multibody kinematics.
        // ---------------------
        derivatives[m_i_x] = vx;
        derivatives[m_i_y] = vy;

        // Multibody dynamics.
        // -------------------
        const auto netForce = calcNetForce(states, controls);
        derivatives[m_i_vx] = netForce.x / mass;
        derivatives[m_i_vy] = netForce.y / mass - ACCEL_GRAVITY;
    }
    NetForce calcNetForce(const mesh::VectorX<T>& states,
                          const mesh::VectorX<T>& controls) const {
        const T& x = states[m_i_x];
        const T& y = states[m_i_y];
        const T& vx = states[m_i_vx];
        const T& vy = states[m_i_vy];

        const T& activationL = controls[m_i_activation_l];
        const T musTenLenL = sqrt(pow(d + x, 2) + pow(y, 2));
        const T musTenVelL = ((d + x) * vx + pow(vy, 2)) / musTenLenL;
        const T tensionL = m_muscleL.calcRigidTendonFiberForceAlongTendon(
                activationL, musTenLenL, musTenVelL);

        const T& activationR = controls[m_i_activation_r];
        const T musTenLenR = sqrt(pow(d - x, 2) + pow(y, 2));
        const T musTenVelR = (-(d - x) * vx + pow(vy, 2)) / musTenLenR;
        const T tensionR = m_muscleR.calcRigidTendonFiberForceAlongTendon(
                activationR, musTenLenR, -musTenVelR);

        const T netForceX = -tensionL * (d + x) / musTenLenL
                            +tensionR * (d - x) / musTenLenR;
        const T netForceY = +tensionL * (-y) / musTenLenL
                            +tensionR * (-y) / musTenLenR;
        return {netForceX,  netForceY};
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

OpenSim::Model buildModel() {
    using SimTK::Vec3;

    Model model;
    //model.setUseVisualizer(true);
    model.setName("block2musc2dof");
    model.set_gravity(Vec3(0, -ACCEL_GRAVITY, 0));

    // Massless intermediate body.
    auto* intermed = new Body("intermed", 0, Vec3(0), SimTK::Inertia(0));
    model.addComponent(intermed);
    auto* body = new Body("body", 1, Vec3(0), SimTK::Inertia(1));
    model.addComponent(body);

    // TODO GSO and MRS do not support locked coordinates yet.
    // Allow translation along x and y; disable rotation about z.
    // TODO auto* joint = new PlanarJoint();
    // TODO joint->setName("joint");
    // TODO joint->connectSocket_parent_frame(model.getGround());
    // TODO joint->connectSocket_child_frame(*body);
    // TODO auto& coordTX = joint->updCoordinate(PlanarJoint::Coord::TranslationX);
    // TODO coordTX.setName("tx");
    // TODO auto& coordTY = joint->updCoordinate(PlanarJoint::Coord::TranslationY);
    // TODO coordTY.setName("ty");
    // TODO auto& coordRZ = joint->updCoordinate(PlanarJoint::Coord::RotationZ);
    // TODO coordRZ.setName("rz");
    // TODO coordRZ.setDefaultLocked(true);
    // TODO model.addComponent(joint);
    auto* jointX = new SliderJoint();
    jointX->setName("tx");
    jointX->connectSocket_parent_frame(model.getGround());
    jointX->connectSocket_child_frame(*intermed);
    auto& coordX = jointX->updCoordinate(SliderJoint::Coord::TranslationX);
    coordX.setName("tx");
    model.addComponent(jointX);

    // The joint's x axis must point in the global "-y" direction.
    auto* jointY = new SliderJoint("ty",
            *intermed, Vec3(0), Vec3(-0.5 * SimTK::Pi, 0, 0),
            *body, Vec3(0), Vec3(-0.5 * SimTK::Pi, 0, 0));
    auto& coordY = jointY->updCoordinate(SliderJoint::Coord::TranslationX);
    coordY.setName("ty");
    model.addComponent(jointY);

    {
        auto* actuL = new Millard2012EquilibriumMuscle();
        actuL->setName("left");
        actuL->set_max_isometric_force(20);
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
        actuR->set_max_isometric_force(20);
        actuR->set_optimal_fiber_length(.20); // TODO change these properties.
        actuR->set_tendon_slack_length(0.10);
        actuR->set_pennation_angle_at_optimal(0.0);
        actuR->addNewPathPoint("origin", model.updGround(),
                               Vec3(+WIDTH, 0, 0));
        actuR->addNewPathPoint("insertion", *body, Vec3(0));
        model.addComponent(actuR);
    }

    //SimTK::State s = model.initSystem();
    //model.updMatterSubsystem().setShowDefaultGeometry(true);
    //model.updVisualizer().updSimbodyVisualizer().setBackgroundType(
    //        SimTK::Visualizer::GroundAndSky);
    //model.getVisualizer().show(s);
    //std::cin.get();
    //Manager manager(model);
    //manager.integrate(s, 1.0);
    //std::cin.get();
    return model;
}

std::pair<TimeSeriesTable, TimeSeriesTable>
solveForTrajectoryGlobalStaticOptimizationSolver(const Model& model) {

    // Solve a trajectory optimization problem.
    auto ocp = std::make_shared<OCPStatic<adouble>>(model);
    ocp->print_description();
    const int N = 100;
    mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal",
                                                  "ipopt", N);
    mesh::OptimalControlSolution ocp_solution = dircol.solve();
    std::string trajectoryFile =
            "test2Muscles2DOFsDeGroote2016_GSO_trajectory.csv";
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
    kinematics.setColumnLabels(
            {"tx/tx/value", "tx/tx/speed", "ty/ty/value", "ty/ty/speed"});
    const auto& x = ocpSolution.getDependentColumn("x");
    const auto& vx = ocpSolution.getDependentColumn("vx");
    const auto& y = ocpSolution.getDependentColumn("y");
    const auto& vy = ocpSolution.getDependentColumn("vy");
    SimTK::RowVector row(4);
    for (size_t iRow = 0; iRow < ocpSolution.getNumRows(); ++iRow) {
        row[0] = x[iRow];
        row[1] = vx[iRow];
        row[2] = y[iRow];
        row[3] = vy[iRow];
        kinematics.appendRow(ocpSolution.getIndependentColumn()[iRow], row);
    }

    // Compute actual inverse dynamics moment, for debugging.
    // ------------------------------------------------------
    // TODO

    return {ocpSolution, kinematics};
}

void test2Muscles2DOFsGlobalStaticOptimizationSolver(
        const std::pair<TimeSeriesTable, TimeSeriesTable>& data,
        const Model& model) {
    const auto& ocpSolution = data.first;
    const auto& kinematics = data.second;

    GlobalStaticOptimizationSolver gso;
    gso.setModel(model);
    gso.setKinematicsData(kinematics);
    gso.set_lowpass_cutoff_frequency_for_joint_moments(50);
    double reserveOptimalForce = 0.001;
    gso.set_create_reserve_actuators(reserveOptimalForce);
    GlobalStaticOptimizationSolver::Solution solution = gso.solve();
    solution.write("test2Muscles2DOFsDeGroote2016_GSO");

    // Compare the solution to the initial trajectory optimization solution.
    // ---------------------------------------------------------------------
    // TODO

}

int main() {
    SimTK_START_TEST("test2Muscles2DOFsDeGroote2016");
        Model model = buildModel();
        model.finalizeFromProperties();
        {
            auto gsoData =
                    solveForTrajectoryGlobalStaticOptimizationSolver(model);
            SimTK_SUBTEST2(test2Muscles2DOFsGlobalStaticOptimizationSolver,
                           gsoData, model);
        }
    SimTK_END_TEST();
}
