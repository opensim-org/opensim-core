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
    inverse.set_output_paths(0, ".*tendon_force.*");
    inverse.set_output_paths(1, ".*fiber_force_along_tendon.*");

    SECTION("Base problem") {
        MocoInverseSolution inverseSolution = inverse.solve();
        MocoSolution solution = inverseSolution.getMocoSolution();
        //solution.write("testMocoInverse_subject_18musc_solution.sto");

        MocoTrajectory std("std_testMocoInverse_subject_18musc_solution.sto");
        const auto expected = std.getControlsTrajectory();
        CHECK(std.compareContinuousVariablesRMS(solution,
                                                {{"controls", {}}}) < 1e-2);
        CHECK(std.compareContinuousVariablesRMS(solution, {{"states", {}}}) < 1e-2);

        // Check muscle-tendon equilibrium.
        TimeSeriesTable outputs = inverseSolution.getOutputs();
        Model model = modelProcessor.process();
        const auto& muscles = model.getComponentList<DeGrooteFregly2016Muscle>();
        for (const auto& muscle : muscles) {
            const auto& path = muscle.getAbsolutePathString();
            const auto& activeFiberForceAlongTendon = outputs.getDependentColumn(
                    path + "|active_fiber_force_along_tendon");
            const auto& passiveFiberForceAlongTendon =
                    outputs.getDependentColumn(
                        path + "|passive_fiber_force_along_tendon");
            const auto& tendonForce = outputs.getDependentColumn(
                    path + "|tendon_force");

            const auto residual = (tendonForce -
                    (passiveFiberForceAlongTendon + activeFiberForceAlongTendon))
                            / muscle.getMaxIsometricForce();

            CHECK(residual.normRMS() < 1e-6);
        }
    }

    SECTION("With a MocoControlBoundConstraint") {
        MocoStudy study = inverse.initialize();
        auto& problem = study.updProblem();

        // Add control bound constraint.
        auto* controlBound = problem.addPathConstraint<MocoControlBoundConstraint>();
        controlBound->addControlPath("/forceset/med_gas_r");
        controlBound->addControlPath("/forceset/med_gas_l");
        controlBound->setLowerBound(Constant(0));
        controlBound->setUpperBound(Constant(0.1));

        auto& solver = study.updSolver<MocoCasADiSolver>();
        solver.resetProblem(problem);
        solver.set_enforce_path_constraint_midpoints(true);

        MocoSolution solution = study.solve();

        auto med_gas_r_excitation = solution.getControl("/forceset/med_gas_r");
        auto med_gas_l_excitation = solution.getControl("/forceset/med_gas_l");

        for (int i = 0; i < solution.getNumTimes(); ++i) {
            CHECK(med_gas_r_excitation[i] < 0.1);
            CHECK(med_gas_l_excitation[i] < 0.1);
        }

    }
}
// Next test_case fails on linux while parsing .sto file, disabling for now 
#ifdef _WIN32
TEST_CASE("Test IMUDataReporter for gait") {

    // Compute accelerometer signals from MocoInverse solution.
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
    MocoTrajectory std("std_testMocoInverse_subject_18musc_solution.sto");
    TimeSeriesTableVec3 accelSignals =
            analyzeMocoTrajectory<SimTK::Vec3>(model, std,
                                               {".*accelerometer_signal"});
    STOFileAdapter_<SimTK::Vec3>::write(accelSignals,
        "testMocoInverse_accelerometer_signals.sto");

    // Compute accelerometer signals with no forces and a PositionMotion created
    // from the model coordinates.

    // Create the coordinates table (in radians).
    auto tableProcessor =
            TableProcessor("subject_walk_armless_coordinates.mot") |
            TabOpLowPassFilter(6) |
            TabOpUseAbsoluteStateNames();
    auto coordinatesRadians = tableProcessor.processAndConvertToRadians(model);
    for (const auto& label : coordinatesRadians.getColumnLabels()) {
        if (label.find("/jointset/") != std::string::npos) {
            std::string speedLabel(label);
            speedLabel.replace(speedLabel.end()-6, speedLabel.end(), "/speed");
            coordinatesRadians.appendColumn(speedLabel,
                coordinatesRadians.getDependentColumn(label));
        } else {
            coordinatesRadians.removeColumn(label);
        }
    }
    STOFileAdapter::write(coordinatesRadians,
                          "subject_walk_armless_coordinates_radians.sto");
    // Create a model with no muscles (or other forces) and add IMU components.
    ModelProcessor modelProcessorNoMuscles =
        ModelProcessor("subject_walk_armless_18musc.osim") |
        ModOpReplaceJointsWithWelds({"subtalar_r", "subtalar_l",
            "mtp_r", "mtp_l"}) |
        ModOpRemoveMuscles() |
        ModOpAddExternalLoads("subject_walk_armless_external_loads.xml");
    Model modelNoMuscles = modelProcessorNoMuscles.process();
    modelNoMuscles.updForceSet().clearAndDestroy();
    OpenSenseUtilities().addModelIMUs(modelNoMuscles, paths);
    modelNoMuscles.initSystem();
    modelNoMuscles.print("subject_walk_armless_with_imus.osim");

    // Add a PositionMotion to the model based on the coordinate trajectories.
    auto statesTraj = StatesTrajectory::createFromStatesTable(model,
          coordinatesRadians, true, true, true);
    auto posmot = PositionMotion::createFromStatesTrajectory(model, statesTraj);
    posmot->setName("position_motion");
    modelNoMuscles.addComponent(posmot.release());
    auto state = modelNoMuscles.initSystem();

    TimeSeriesTable empty(coordinatesRadians.getIndependentColumn());
    TimeSeriesTableVec3 accelSignalsNoMuscles =
            analyze<SimTK::Vec3>(modelNoMuscles, coordinatesRadians, empty,
                                           {".*accelerometer_signal"});

    // Resample the accelerometer signals so we can compare to the MocoInverse
    // solution accelerometer signals.
    TimeSeriesTable accelSignalsNoMusclesFlat = accelSignalsNoMuscles.flatten();
    GCVSplineSet accelSplines(accelSignalsNoMusclesFlat);
    const auto& time = accelSignals.getIndependentColumn();
    TimeSeriesTable accelSignalsNoMusclesDownSampledFlat(time);
    for (const auto& label : accelSignalsNoMusclesFlat.getColumnLabels()) {
        SimTK::Vector col((int)time.size(), 0.0);
        const auto& thisSpline = accelSplines.get(label);
        for (int i = 0; i < (int)time.size(); ++i) {
            SimTK::Vector timeVec(1, time[i]);
            col[i] = thisSpline.calcValue(timeVec);
        }
        accelSignalsNoMusclesDownSampledFlat.appendColumn(label, col);
    }
    accelSignalsNoMusclesDownSampledFlat.addTableMetaData<std::string>(
            "inDegrees", "no");
    STOFileAdapter::write(accelSignalsNoMusclesDownSampledFlat,
                    "testMocoInverse_no_forces_linear_accelerations.sto");

    // Compute error in accelerometer signals.
    auto accelSignalsFlat = accelSignals.flatten();
    auto accelBlock = accelSignalsFlat.getMatrixBlock(0, 0,
        accelSignalsFlat.getNumRows(),
        accelSignalsFlat.getNumColumns());
    auto accelBlockNoMuscles =
        accelSignalsNoMusclesDownSampledFlat.getMatrixBlock(0, 0,
        accelSignalsNoMusclesDownSampledFlat.getNumRows(),
        accelSignalsNoMusclesDownSampledFlat.getNumColumns());
    auto diff = accelBlock - accelBlockNoMuscles;
    auto diffSqr = diff.elementwiseMultiply(diff);
    auto sumSquaredError = diffSqr.rowSum();
    // Trapezoidal rule for uniform grid:
    // dt / 2 (f_0 + 2f_1 + 2f_2 + 2f_3 + ... + 2f_{N-1} + f_N)
    double timeInterval = time[(int)time.size()-1] - time[0];
    int numTimes = (int)time.size();
    auto integratedSumSquaredError = timeInterval / 2.0 *
               (sumSquaredError.sum() +
                sumSquaredError(1, numTimes - 2).sum());
    SimTK_TEST_EQ_TOL(integratedSumSquaredError, 0.0, 1e-4);

    // Redo the test above, but now using the AnalyzeTool interface.
    AnalyzeTool analyzeIMU;
    analyzeIMU.setName("testMocoInverse_analysis");
    analyzeIMU.setModelFilename("subject_walk_armless_with_imus.osim");
    analyzeIMU.setCoordinatesFileName(
            "subject_walk_armless_coordinates_radians.sto");
    analyzeIMU.setLowpassCutoffFrequency(-1);
    IMUDataReporter imuDataReporter;
    imuDataReporter.setName("IMUDataReporter_no_forces");
    imuDataReporter.set_compute_accelerations_without_forces(true);
    imuDataReporter.setInDegrees(false);
    analyzeIMU.updAnalysisSet().cloneAndAppend(imuDataReporter);
    analyzeIMU.print("testMocoInverse_analyze_imu_accel.xml");
    AnalyzeTool roundTrip("testMocoInverse_analyze_imu_accel.xml");
    roundTrip.run();

    // Compare the original accelerations to the accelerations computed with
    // AnalyzeTool.
    auto accelSignalsNoMusclesAnalyzeTool =
            TimeSeriesTableVec3(
                    "testMocoInverse_analysis_linear_accelerations.sto");

    // Resample the accelerometer signals so we can compare to the MocoInverse
    // solution accelerometer signals.
    TimeSeriesTable accelSignalsNoMusclesAnalyzeToolFlat =
            accelSignalsNoMusclesAnalyzeTool.flatten();
    GCVSplineSet accelSplinesAnalyzeTool(accelSignalsNoMusclesAnalyzeToolFlat);
    TimeSeriesTable accelSignalsNoMusclesAnalyzeToolDownSampledFlat(time);
    for (const auto& label : accelSignalsNoMusclesAnalyzeToolFlat.getColumnLabels()) {
        SimTK::Vector col((int)time.size(), 0.0);
        const auto& thisSpline = accelSplinesAnalyzeTool.get(label);
        for (int i = 0; i < (int)time.size(); ++i) {
            SimTK::Vector timeVec(1, time[i]);
            col[i] = thisSpline.calcValue(timeVec);
        }
        accelSignalsNoMusclesAnalyzeToolDownSampledFlat.appendColumn(label, col);
    }
    accelSignalsNoMusclesAnalyzeToolDownSampledFlat.addTableMetaData<std::string>(
            "inDegrees", "no");

    // Compute accelerometer signal errors.
    auto accelBlockNoMusclesAnalyzeTool =
        accelSignalsNoMusclesAnalyzeToolDownSampledFlat.getMatrixBlock(0, 0,
        accelSignalsNoMusclesAnalyzeToolDownSampledFlat.getNumRows(),
        accelSignalsNoMusclesAnalyzeToolDownSampledFlat.getNumColumns());
    auto diffAnalyze = accelBlock - accelBlockNoMusclesAnalyzeTool;
    auto diffSqrAnalyze = diffAnalyze.elementwiseMultiply(diffAnalyze);
    auto sumSquaredErrorAnalyze = diffSqrAnalyze.rowSum();
    auto integratedSumSquaredErrorAnalyze = timeInterval / 2.0 *
               (sumSquaredErrorAnalyze.sum() +
                sumSquaredErrorAnalyze(1, numTimes - 2).sum());
    // TODO The error when using AnalyzeTool is slightly larger.
    SimTK_TEST_EQ_TOL(integratedSumSquaredErrorAnalyze, 0.0, 1e-3);
}
#endif
