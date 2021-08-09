/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoInverse.cpp                                          *
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

#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Analyses/IMUDataReporter.h>

#define CATCH_CONFIG_MAIN
#include "Testing.h"

using namespace OpenSim;

/// This test case helps debug when it's necessary to call prescribe() versus
/// realize().
TEST_CASE("PrescribedKinematics prescribe() and realize()") {
    Model model = ModelFactory::createPendulum();
    auto* motion = new PositionMotion();
    const double c2 = 1.3;
    const double c1 = 0.17;
    const double c0 = 0.81;
    motion->setPositionForCoordinate(model.getCoordinateSet().get(0),
            PolynomialFunction(createVector({c2, c1, c0})));
    model.addModelComponent(motion);
    auto state = model.initSystem();

    const auto& system = model.getSystem();
    const auto& y = state.getY();
    const auto& ydot = state.getYDot();
    CHECK(y[0] == 0);
    CHECK(y[1] == 0);
    system.prescribe(state);
    CHECK(y[0] == Approx(c0));
    CHECK(y[1] == Approx(c1));
    model.realizeAcceleration(state);
    CHECK(y[0] == Approx(c0));
    CHECK(y[1] == Approx(c1));
    CHECK(ydot[0] == Approx(c1));
    CHECK(ydot[1] == Approx(2 * c2));

    const double t = 0.3;
    state.setTime(t);
    system.prescribe(state);
    CHECK(y[0] == Approx(c0 + c1 * t + c2 * pow(t, 2)));
    CHECK(y[1] == Approx(c1 + 2 * c2 * t));
    model.realizeAcceleration(state);
    CHECK(y[0] == Approx(c0 + c1 * t + c2 * pow(t, 2)));
    CHECK(y[1] == Approx(c1 + 2 * c2 * t));
    CHECK(ydot[0] == Approx(c1 + 2 * c2 * t));
    CHECK(ydot[1] == Approx(2 * c2));
}

TEST_CASE("PrescribedKinematics direct collocation auxiliary dynamics",
        "[casadi]") {

    // Make sure that custom dynamics are still handled properly even when
    // we are skipping over the kinematic/multibody states. That is, this test
    // ensures we handle indices properly.
    class CustomDynamics : public Component {
        OpenSim_DECLARE_CONCRETE_OBJECT(CustomDynamics, Component);

    public:
        OpenSim_DECLARE_PROPERTY(
                default_s, double, "Default state variable value.");
        CustomDynamics() { constructProperty_default_s(0.4361); }
        void extendAddToSystem(SimTK::MultibodySystem& system) const override {
            Super::extendAddToSystem(system);
            addStateVariable("s");
        }
        void extendInitStateFromProperties(SimTK::State& s) const override {
            Super::extendInitStateFromProperties(s);
            setStateVariableValue(s, "s", get_default_s());
        }
        void extendSetPropertiesFromState(const SimTK::State& s) override {
            Super::extendSetPropertiesFromState(s);
            set_default_s(getStateVariableValue(s, "s"));
        }
        void computeStateVariableDerivatives(
                const SimTK::State& s) const override {
            setStateVariableDerivativeValue(
                    s, "s", getStateVariableValue(s, "s"));
        }
    };
    Model model = ModelFactory::createPendulum();
    model.initSystem();
    auto* motion = new PositionMotion();
    motion->setPositionForCoordinate(
            model.getCoordinateSet().get(0), LinearFunction(1.3, 0.17));
    model.addModelComponent(motion);
    model.addComponent(new CustomDynamics());

    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelAsCopy(model);
    problem.setTimeBounds(0, 1);
    const double init_s = 0.2;
    problem.setStateInfo("/customdynamics/s", {0, 100}, init_s);
    auto& solver = study.initCasADiSolver();
    solver.set_transcription_scheme("trapezoidal");
    solver.set_multibody_dynamics_mode("implicit");

    MocoSolution solution = study.solve();
    OpenSim_CHECK_MATRIX_TOL(solution.getState("/customdynamics/s"),
            0.2 * SimTK::exp(solution.getTime()), 1e-4);
}

TEST_CASE("MocoInverse Rajagopal2016, 18 muscles", "[casadi]") {

    MocoInverse inverse;
    ModelProcessor modelProcessor =
        ModelProcessor("subject_walk_armless_18musc.osim") |
        ModOpReplaceJointsWithWelds({"subtalar_r", "subtalar_l",
            "mtp_r", "mtp_l"}) |
        ModOpReplaceMusclesWithDeGrooteFregly2016() |
        ModOpIgnorePassiveFiberForcesDGF() |
        ModOpTendonComplianceDynamicsModeDGF("implicit") |
        ModOpAddExternalLoads("subject_walk_armless_external_loads.xml");

    inverse.setModel(modelProcessor);
    inverse.setKinematics(
        TableProcessor("subject_walk_armless_coordinates.mot") |
        TabOpLowPassFilter(6));
    inverse.set_initial_time(0.450);
    inverse.set_final_time(1.0);
    inverse.set_kinematics_allow_extra_columns(true);
    inverse.set_mesh_interval(0.025);
    inverse.set_constraint_tolerance(1e-4);
    inverse.set_convergence_tolerance(1e-4);

    MocoSolution solution = inverse.solve().getMocoSolution();
    //solution.write("testMocoInverse_subject_18musc_solution.sto");

    MocoTrajectory std("std_testMocoInverse_subject_18musc_solution.sto");
    const auto expected = std.getControlsTrajectory();
    CHECK(std.compareContinuousVariablesRMS(solution,
            {{"controls", {}}}) < 1e-2);
    CHECK(std.compareContinuousVariablesRMS(solution, {{"states", {}}}) < 1e-2);
}

TEST_CASE("Test IMUDataReporter for gait") {

    ModelProcessor modelProcessor =
        ModelProcessor("subject_walk_armless_18musc.osim") |
        ModOpReplaceJointsWithWelds({"subtalar_r", "subtalar_l",
            "mtp_r", "mtp_l"}) |
        ModOpReplaceMusclesWithDeGrooteFregly2016() |
        ModOpIgnorePassiveFiberForcesDGF() |
        ModOpTendonComplianceDynamicsModeDGF("implicit") |
        ModOpAddExternalLoads("subject_walk_armless_external_loads.xml");

    std::vector<std::string> paths = {"/bodyset/pelvis",
                                       "/bodyset/femur_r",
                                       "/bodyset/tibia_r",
                                       "/bodyset/calcn_r"};

    Model model = modelProcessor.process();
    OpenSenseUtilities().addModelIMUs(model, paths);
    model.initSystem();
    model.print("subject_walk_armless_18musc_with_imus.osim");

    auto tableProcessor =
            TableProcessor("subject_walk_armless_coordinates.mot") |
            TabOpLowPassFilter(6);
    STOFileAdapter::write(tableProcessor.processAndConvertToRadians(model),
        "subject_walk_armless_coordinates_radians.sto");


    MocoTrajectory std("std_testMocoInverse_subject_18musc_solution.sto");
    TimeSeriesTable stdTable("std_testMocoInverse_subject_18musc_solution.sto");


    for (const auto& label : stdTable.getColumnLabels()) {
        if (label.find("forceset") != std::string::npos ||
                label.find("lambda") != std::string::npos) {
            stdTable.removeColumn(label);
        }
    }
    STOFileAdapter::write(stdTable,
                          "testMocoInverse_solution_kinematics_states.sto");

    Storage coordinates("testMocoInverse_solution_kinematics_states.sto");

    TimeSeriesTableVec3 accelSignals =
            analyzeMocoTrajectory<SimTK::Vec3>(model, std,
                                               {".*accelerometer_signal"});
    STOFileAdapter_<SimTK::Vec3>::write(accelSignals,
            "testMocoInverse_accelerometer_signals.sto");

    ModelProcessor modelProcNoMuscles =
        ModelProcessor("subject_walk_armless_18musc.osim") |
        ModOpReplaceJointsWithWelds({"subtalar_r", "subtalar_l",
            "mtp_r", "mtp_l"}) |
        ModOpRemoveMuscles() |
        ModOpAddExternalLoads("subject_walk_armless_external_loads.xml");

    Model modelNoMuscles = modelProcNoMuscles.process();
    modelNoMuscles.updForceSet().clearAndDestroy();
    OpenSenseUtilities().addModelIMUs(modelNoMuscles, paths);
    modelNoMuscles.initSystem();

    auto statesTraj = StatesTrajectory::createFromStatesTable(model, stdTable,
            true, true, true);

    auto posmot = PositionMotion::createFromStatesTrajectory(model, statesTraj);
    posmot->setName("position_motion");
    modelNoMuscles.addComponent(posmot.release());
    auto state = modelNoMuscles.initSystem();
    std::cout << "DEBUG numControls: " << modelNoMuscles.getNumControls() << std::endl;

    TimeSeriesTable emptyControlsTable(stdTable.getIndependentColumn());
    TimeSeriesTableVec3 accelSignalsNoMuscles =
            analyze<SimTK::Vec3>(modelNoMuscles, stdTable, emptyControlsTable,
                                               {".*accelerometer_signal"});

    STOFileAdapter_<SimTK::Vec3>::write(accelSignalsNoMuscles,
                    "testMocoInverse_no_forces_linear_accelerations.sto");

//    modelNoMuscles.print("subject_walk_armless_with_imus.osim");
//
//    AnalyzeTool analyzeIMU;
//    analyzeIMU.setName("testMocoInverse_analysis");
//    analyzeIMU.setModelFilename("subject_walk_armless_with_imus.osim");
//    analyzeIMU.setStatesFileName("testMocoInverse_solution_kinematics_states.sto");
//    analyzeIMU.setStartTime(0.450);
//    analyzeIMU.setFinalTime(1.0);
//    analyzeIMU.setLowpassCutoffFrequency(-1);
//    IMUDataReporter imuDataReporter;
//    imuDataReporter.setName("IMUDataReporter_no_forces");
//    imuDataReporter.set_compute_accelerations_without_forces(true);
//    imuDataReporter.setInDegrees(false);
//    analyzeIMU.updAnalysisSet().cloneAndAppend(imuDataReporter);
//    analyzeIMU.print("testMocoInverse_analyze_imu_accel.xml");
//    AnalyzeTool roundTrip("testMocoInverse_analyze_imu_accel.xml");
//    roundTrip.run();

    // Compare the original accelerations to the accelerations computed with
    // AnalyzeTool.
    auto accelBlock = accelSignals.getMatrixBlock(0, 0,
        accelSignals.getNumRows(),
        accelSignals.getNumColumns());
    auto accelSignals_no_forces =
            TimeSeriesTableVec3(
                    "testMocoInverse_analysis_linear_accelerations.sto");
    auto accelBlockNoForces = accelSignalsNoMuscles.getMatrixBlock(0, 0,
        accelSignalsNoMuscles.getNumRows(),
        accelSignalsNoMuscles.getNumColumns());

    auto diff = accelBlock - accelBlockNoForces;
    std::cout << "DEBUG: " << diff << std::endl;
    SimTK_TEST_EQ_TOL(accelBlock, accelBlockNoForces, 1e-3);
}
