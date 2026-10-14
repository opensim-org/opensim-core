/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testAnalyzeTool.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Ajay Seth                                          *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// INCLUDE
#include <OpenSim/Common/CSVFileAdapter.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Analyses/OutputReporter.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/SimulationUtilities.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>
#include <OpenSim/Analyses/IMUDataReporter.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Simulation/Control/PrescribedController.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/GCVSplineSet.h>

#include <tests/Testing.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;
using namespace std;

namespace {

    // Test different default activations are respected when activation
    // states are not provided.
    void testTugOfWar(const string& dataFileName, const double& defaultAct) {
        AnalyzeTool analyze("Tug_of_War_Setup_Analyze.xml");
        analyze.setCoordinatesFileName("");
        analyze.setStatesFileName("");

        // Access the model and muscle being analyzed
        Model& model = analyze.getModel();
        Millard2012EquilibriumMuscle& muscle =
            static_cast<Millard2012EquilibriumMuscle&>(model.updMuscles()[0]);
        bool isCoordinatesOnly = true;
        // Load in the States used to recompute the results of the Analysis
        Storage dataStore(dataFileName);
        if (dataStore.getColumnLabels().findIndex(muscle.getName() + ".activation") > 0) {
            isCoordinatesOnly = false;
            analyze.setStatesFileName(dataFileName);
            // ramp input starts at 0.0
            muscle.set_minimum_activation(0.0);
        }
        else {
            analyze.setCoordinatesFileName(dataFileName);
        }

        // Test that the default activation is taken into consideration by
        // the Analysis and reflected in the AnalyzeTool solution
        muscle.set_default_activation(defaultAct);

        analyze.run();

        // Load the AnalyzTool results for the muscle's force through time
        TimeSeriesTable_<double> results(
            "Analyze_Tug_of_War/Tug_of_War_Millard_Iso_ForceReporter_forces.sto");
        assert(results.getNumColumns() == 1);
        SimTK::Vector forces = results.getDependentColumnAtIndex(0);

        TimeSeriesTable_<double> outputs_table(
            "Analyze_Tug_of_War/Tug_of_War_Millard_Iso_Outputs.sto");
        SimTK::Vector tf_output = outputs_table.getDependentColumnAtIndex(1);

        // Load input data as StatesTrajectory used to perform the Analysis
        auto statesTraj = StatesTrajectory::createFromStatesStorage(
            model, dataStore, true, false);
        size_t nstates = statesTraj.getSize();

        // muscle active, passive, total muscle and tendon force quantities
        double af, pf, mf, tf, fl;
        af= pf = mf = tf = fl = SimTK::NaN;

        // Tolerance for muscle equilibrium solution
        const double equilTol = muscle.getMaxIsometricForce()*SimTK::SqrtEps;

        // The maximum acceptable change in force between two contiguous states
        const double maxDelta = muscle.getMaxIsometricForce() / 10;

        SimTK::State s = model.getWorkingState();
        // Independently compute the active fiber force at every state
        for (size_t i = 0; i < nstates; ++i) {
            s = statesTraj[i];
            // When the muscle states are not supplied in the input dataStore
            // (isCoordinatesOnly == true), then set it to its default value.
            if (isCoordinatesOnly) {
                muscle.setActivation(s, muscle.get_default_activation());
            }
            // technically, fiber lengths could be supplied, but this test case
            // (a typical use case) does not and therefore set to its default.
            muscle.setFiberLength(s, muscle.get_default_fiber_length());
            try {
                muscle.computeEquilibrium(s);
            }
            catch (const MuscleCannotEquilibrate& x) {
                // Write out the muscle equilibrium for error as a function of
                // fiber-length.
                OpenSim::Testing::reportTendonAndFiberForcesAcrossFiberLengths(muscle, s);
                throw x;
            }
            model.realizeDynamics(s);

            // Get the fiber-length
            fl = muscle.getFiberLength(s);

            cout << "t = " << s.getTime() << " | fiber_length = " << fl <<
            " : default_fiber_length = " << muscle.get_default_fiber_length() << endl;

            SimTK_ASSERT_ALWAYS(fl >= muscle.getMinimumFiberLength(),
                "Equilibrium failed to compute valid fiber length.");

            if (isCoordinatesOnly) {
                // check that activation is not reset to zero or other value
                SimTK_ASSERT_ALWAYS(muscle.getActivation(s) == defaultAct,
                    "Test failed to correctly use the default activation value.");
            }
            else {
                // check that activation used was that supplied by the dataStore
                SimTK_ASSERT_ALWAYS(muscle.getActivation(s) == s.getTime(),
                    "Test failed to correctly use the supplied activation values.");
            }

            // get active and passive forces given the default activation
            af = muscle.getActiveFiberForceAlongTendon(s);
            pf = muscle.getPassiveFiberForceAlongTendon(s);

            // now the total muscle force is the active + passive
            mf = af + pf;
            tf = muscle.getTendonForce(s);

            // equilibrium demands tendon and muscle fiber are equivalent
            OpenSim_CHECK_EQUAL(tf, mf, equilTol);
            // Verify that the current computed and AnalyzeTool reported force are
            // equivalent for the provided motion file
            cout << s.getTime() << " :: muscle-fiber-force: " << mf <<
                " Analyze reported force: " << forces[int(i)] << endl;
            OpenSim_CHECK_EQUAL(mf, forces[int(i)], equilTol);

            cout << s.getTime() << " :: tendon-force: " << tf <<
                " Analyze Output reported: " << tf_output[int(i)] << endl;
            OpenSim_CHECK_EQUAL(tf, tf_output[int(i)], equilTol);

            double delta = (i > 0) ? abs(forces[int(i)]-forces[int(i-1)]) : 0;

            SimTK_ASSERT_ALWAYS(delta < maxDelta,
                "Force trajectory has unexplained discontinuity.");
        }
    }

}

TEST_CASE("testTugOfWar CoordinatesOnly: default_act = 0.01") {
    testTugOfWar("Tug_of_War_ConstantVelocity.sto", 0.01);
}

TEST_CASE("testTugOfWar CoordinatesOnly: default_act = 1.0") {
    testTugOfWar("Tug_of_War_ConstantVelocity.sto", 1.0);
}

TEST_CASE("testTugOfWar with activation state provided") {
     testTugOfWar("Tug_of_War_ConstantVelocity_RampActivation.sto", 0.0);
}

TEST_CASE("testTutorialOne") {
    AnalyzeTool analyze1("PlotterTool.xml");
    analyze1.getModel().print("testAnalyzeTutorialOne.osim");
    analyze1.run();
    /* Once this runs to completion we'll make the test more meaningful by comparing output
    * to a validated standard. Let's make sure we don't crash during run first! -Ayman 5/29/12 */
    Storage resultFiberLength("testPlotterTool/BothLegs__FiberLength.sto");
    Storage standardFiberLength("std_BothLegs_fiberLength.sto");
    CHECK_STORAGE_AGAINST_STANDARD(resultFiberLength, standardFiberLength,
        std::vector<double>(100, 0.0001));
    // const Model& mdl = analyze1.getModel();
    //mdl.updMultibodySystem()
    analyze1.setStatesFileName("plotterGeneratedStatesHip45.sto");
    //analyze1.setModel(mdl);
    analyze1.setName("BothLegsHip45");
    analyze1.run();
    Storage resultFiberLengthHip45("testPlotterTool/BothLegsHip45__FiberLength.sto");
    Storage standardFiberLength45("std_BothLegsHip45__FiberLength.sto");
    CHECK_STORAGE_AGAINST_STANDARD(resultFiberLengthHip45,
        standardFiberLength45, std::vector<double>(100, 0.0001));
    cout << "testAnalyzeTutorialOne passed" << endl;
}


TEST_CASE("testActuationAnalysisWithDisabledForce") {
    AnalyzeTool analyze("PlotterTool.xml");
    Model& model = analyze.getModel();
    auto& muscle = model.updMuscles()[0];
    muscle.set_appliesForce(false);

    std::string resultsDir = "testPlotterToolWithDisabledForce";
    analyze.setResultsDir(resultsDir);
    analyze.run();

    // Reading a file with mismatched nColumns header and actual number of
    // data columns will throw.
    TimeSeriesTable_<double> act_force_table(
            resultsDir + "/BothLegs_Actuation_force.sto");

    // Let's also check that the number of columns is correct (i.e.,
    // (number of muscles in the model) - 1).
    OpenSim_CHECK_EQUAL(model.getMuscles().getSize() - 1,
            (int)act_force_table.getNumColumns());
}

TEST_CASE("testBodyKinematics") {
    Model model;
    model.setGravity(SimTK::Vec3(0));
    Body* body = new Body("body", 1, SimTK::Vec3(0), SimTK::Inertia(1));
    model.addBody(body);

    // Rotate child frame to align the body's local X axis with the ground's Z
    // axis. We'll apply a simple constant rotation about ground Z below
    // for the test.
    FreeJoint* joint = new FreeJoint("joint",
        model.getGround(), SimTK::Vec3(0), SimTK::Vec3(0),
        *body, SimTK::Vec3(0), SimTK::Vec3(0, SimTK::Pi/2, 0));
    model.addJoint(joint);

    BodyKinematics* bodyKinematicsLocal = new BodyKinematics(&model);
    bodyKinematicsLocal->setName("BodyKinematics_local");
    bodyKinematicsLocal->setExpressResultsInLocalFrame(true);
    bodyKinematicsLocal->setInDegrees(true);

    BodyKinematics* bodyKinematicsGround = new BodyKinematics(&model);
    bodyKinematicsGround->setName("BodyKinematics_ground");
    bodyKinematicsGround->setExpressResultsInLocalFrame(false);
    bodyKinematicsGround->setInDegrees(false);

    model.addAnalysis(bodyKinematicsLocal);
    model.addAnalysis(bodyKinematicsGround);

    SimTK::State& s = model.initSystem();

    // Apply a constnat velocity simple rotation about the ground Z,
    // and translation in the ground X and Y directions
    double speedRot = 1.0;
    double speedX = 2.0;
    double speedY = 3.0;
    joint->updCoordinate(FreeJoint::Coord::Rotation3Z)
            .setSpeedValue(s, speedRot);
    joint->updCoordinate(FreeJoint::Coord::TranslationX)
            .setSpeedValue(s, speedX);
    joint->updCoordinate(FreeJoint::Coord::TranslationY)
            .setSpeedValue(s, speedY);

    Manager manager(model);
    double duration = 2.0;
    manager.initialize(s);
    s = manager.integrate(duration);

    bodyKinematicsLocal->printResults("");
    bodyKinematicsGround->printResults("");

    Storage localVel("_BodyKinematics_local_vel_bodyLocal.sto");
    Storage groundVel("_BodyKinematics_ground_vel_global.sto");
    Array<double> localVelOx, localVelOz, groundVelOx, groundVelOz;
    localVel.getDataColumn("body_Ox", localVelOx);
    localVel.getDataColumn("body_Oz", localVelOz);
    groundVel.getDataColumn("body_Ox", groundVelOx);
    groundVel.getDataColumn("body_Oz", groundVelOz);

    // Test rotation was a simple rotation about ground Z, which is aligned
    // with the body X. Also note that local results are printed in degrees,
    // and ground results are printed in radians.
    double tol = 1e-6;
    OpenSim_CHECK_EQUAL(localVelOx.getLast(),
        static_cast<double>(speedRot * SimTK_RADIAN_TO_DEGREE), tol);
    OpenSim_CHECK_EQUAL(localVelOz.getLast(), 0.0, tol);
    OpenSim_CHECK_EQUAL(groundVelOx.getLast(), 0.0, tol);
    OpenSim_CHECK_EQUAL(groundVelOz.getLast(), speedRot, tol);

    Array<double> groundPosX, groundPosY;
    Storage groundPos("_BodyKinematics_ground_pos_global.sto");
    groundPos.getDataColumn("body_X", groundPosX);
    groundPos.getDataColumn("body_Y", groundPosY);
    OpenSim_CHECK_EQUAL(groundPosX.getLast(), speedX * duration, tol);
    OpenSim_CHECK_EQUAL(groundPosY.getLast(), speedY * duration, tol);
}

TEST_CASE("testIMUDataReporter") {
    Model pendulum = ModelFactory::createNLinkPendulum(2);

    BodyKinematics* bodyKinematics = new BodyKinematics(&pendulum);
    bodyKinematics->setName("BodyKinematics_fall");
    bodyKinematics->setRecordCenterOfMass(false);
    bodyKinematics->setExpressResultsInLocalFrame(false);
    bodyKinematics->setInDegrees(false);

    IMUDataReporter* imuDataReporter =
            new IMUDataReporter(&pendulum);
    imuDataReporter->setName("IMU_DataReporter");
    std::vector<std::string> framePaths = {"/bodyset/b0", "/bodyset/b1"};
    imuDataReporter->append_frame_paths("/bodyset/b0");
    imuDataReporter->append_frame_paths("/bodyset/b1");

    pendulum.addAnalysis(bodyKinematics);
    pendulum.addAnalysis(imuDataReporter);

    SimTK::State& s = pendulum.initSystem();

    const Joint& j0 = pendulum.getJointSet()[0];
    const auto& q0 = j0.getCoordinate();
    q0.setValue(s, SimTK::Pi / 2.0); // lowest-point hanging condition

    const Joint& j1 = pendulum.getJointSet()[1];
    const auto& q1 = j1.getCoordinate();
    q1.setValue(s, 0.0);

    Manager manager(pendulum);
    double duration = 2.0;
    manager.initialize(s);
    s = manager.integrate(duration);

    imuDataReporter->printResults("static", "");
    const TimeSeriesTable_<SimTK::Vec3>& angVelTable =
            imuDataReporter->getGyroscopeSignalsTable();
    const TimeSeriesTable_<SimTK::Vec3>& linAccTable =
            imuDataReporter->getAccelerometerSignalsTable();
    const TimeSeriesTable_<SimTK::Quaternion>& rotationsTable =
            imuDataReporter->getOrientationsTable();
    int angNr = int(angVelTable.getNumRows());
    for (int row = 0; row < angNr; ++row) {
        OpenSim_CHECK_EQUAL(angVelTable.getMatrix()[row][0].norm(), 0., 1e-7);
        OpenSim_CHECK_EQUAL(angVelTable.getMatrix()[row][1].norm(), 0., 1e-7);
    }
    // Now allow pendulum to drop under gravity from horizontal
    bodyKinematics->getPositionStorage()->purge();
    q0.setValue(s, 0.0); // Horizontal position
    s.setTime(0.0);
    Manager manager2(pendulum);
    manager2.initialize(s);
    s = manager2.integrate(duration);
    // Compare results to Body kinematics
    auto orientationTableIMU = imuDataReporter->getOrientationsTable();
    auto orientationTableBodyKin = bodyKinematics->getPositionStorage();
    int nr = int(orientationTableIMU.getNumRows());
    for (int row = 0; row < nr; ++row) {
        // fromBodyKin has positions followed by rotations for each body
        Array<double>& fromBodyKin =
                orientationTableBodyKin->getStateVector(row)->getData();
        for (int b = 0; b <= 1; b++) {
            SimTK::Vec3 bodyFixedRotations =
                    SimTK::Rotation(orientationTableIMU.getRowAtIndex(row)[b])
                            .convertRotationToBodyFixedXYZ();
            SimTK::Vec3 fromBodyKinRotations = SimTK::Vec3(&fromBodyKin[b * 6 + 3]);
            OpenSim_CHECK_EQUAL(
                (bodyFixedRotations - fromBodyKinRotations).norm(), 0.0, 1e-7);
        }
    }
    /* Attempt to compare to createSyntheticIMUAccelerationSignals */
    TimeSeriesTable statesTable = manager2.getStatesTable();
    TimeSeriesTable controlsTable(statesTable.getIndependentColumn());
    SimTK::Vector zeroControl(int(controlsTable.getNumRows()), 0.0);
    controlsTable.appendColumn("/tau0", zeroControl);
    controlsTable.appendColumn("/tau1", zeroControl);
    TimeSeriesTableVec3 accelTableFromUtility =
            createSyntheticIMUAccelerationSignals(
                    pendulum, statesTable, controlsTable, framePaths);
    auto diff = (accelTableFromUtility.getMatrix() -
                 imuDataReporter->getAccelerometerSignalsTable().getMatrix());
    auto elemSum = diff.colSum().rowSum().norm();
    OpenSim_CHECK_EQUAL(elemSum, 0.0, 1e-5);

    // Now test AnalyzeTool workflow
    AnalyzeTool analyzeIMU;
    analyzeIMU.setName("dpend_imu");
    analyzeIMU.setModelFilename("double_pendulum.osim");
    analyzeIMU.setCoordinatesFileName("double_pendum1sec.sto");
    IMUDataReporter imuDataReporter2;
    imuDataReporter2.setName("IMU_DataReporter");
    imuDataReporter2.append_frame_paths("/bodyset/rod1/rod1_geom_frame_1");
    imuDataReporter2.append_frame_paths("/bodyset/rod2");
    analyzeIMU.updAnalysisSet().cloneAndAppend(imuDataReporter2);
    analyzeIMU.print("analyzeReportIMUData.xml");
    AnalyzeTool roundTrip("analyzeReportIMUData.xml");
    roundTrip.run();

    // Create another pendulum simulation to test that IMUDataReporter can
    // produce the correct accelerations when the applied forces are unknown.
    {
        // Create a fresh double pendulum model.
        Model pendulum = ModelFactory::createNLinkPendulum(2);

        // Add an IMUDataReporter analysis.
        IMUDataReporter* imuDataReporter = new IMUDataReporter(&pendulum);
        imuDataReporter->setName("IMUDataReporter");
        std::vector<std::string> framePaths = {"/bodyset/b0", "/bodyset/b1"};
        imuDataReporter->append_frame_paths("/bodyset/b0");
        imuDataReporter->append_frame_paths("/bodyset/b1");
        pendulum.addAnalysis(imuDataReporter);

        // Finalize the model system and print the unactuated model to a file.
        // We'll use this model with the AnalyzeTool below.
        auto& state = pendulum.initSystem();
        pendulum.print("testIMUDataReporter_double_pendulum.osim");

        // Add a PrescribedController to the model to control the two torque
        // actuators in the model.
        PrescribedController* controller = new PrescribedController();
        controller->setName("torque_controller");
        controller->addActuator(
                pendulum.getComponent<CoordinateActuator>("/tau0"));
        controller->addActuator(
                pendulum.getComponent<CoordinateActuator>("/tau1"));
        // Specify constant torque functions to the torque actuators
        controller->prescribeControlForActuator("tau0", Constant(10.0));
        controller->prescribeControlForActuator("tau1", Constant(10.0));
        pendulum.addController(controller);
        state = pendulum.initSystem();

        // Set a horizontal default pendulum position.
        const Joint& j0 = pendulum.getJointSet()[0];
        const auto& q0 = j0.getCoordinate();
        const Joint& j1 = pendulum.getJointSet()[1];
        const auto& q1 = j1.getCoordinate();
        q0.setValue(state, 0.0);
        q1.setValue(state, 0.0);

        // Set the initial time and run the integration.
        state.setTime(0.0);
        Manager manager(pendulum);
        manager.setIntegratorMaximumStepSize(1e-3);
        manager.initialize(state);
        manager.integrate(5.0);

        // Extract the accelerometer signals from the torque-driven forward
        // integration.
        TimeSeriesTableVec3 accelSignals =
                imuDataReporter->getAccelerometerSignalsTable();
        accelSignals.trim(2.0, 4.0);
        STOFileAdapter_<SimTK::Vec3>::write(accelSignals,
                              "testIMUDataReporter_linear_accelerations.sto");

        // Save the coordinate states from the forward integration.
        TimeSeriesTable statesTable = manager.getStatesTable();
        STOFileAdapter::write(statesTable, "testIMUDataReporter_states.sto");

        // Construct an AnalyzeTool driven by the coordinate states from the
        // previous forward integration, but without the torque controls. We'll
        // compute the accelerometer signals again, now setting the property
        // 'compute_accelerations_without_forces' on IMUDataReporter to true.
        // This flag will add motion forces to the model to ensure that the
        // correct accelerations are computed.
        AnalyzeTool analyzeIMU;
        analyzeIMU.setName("testIMUDataReporter_no_forces");
        analyzeIMU.setModelFilename("testIMUDataReporter_double_pendulum.osim");
        analyzeIMU.setStatesFileName("testIMUDataReporter_states.sto");
        analyzeIMU.setInitialTime(0.0);
        analyzeIMU.setFinalTime(5.0);
        IMUDataReporter imuDataReporter2;
        imuDataReporter2.setName("IMUDataReporter_no_forces");
        imuDataReporter2.append_frame_paths("/bodyset/b0");
        imuDataReporter2.append_frame_paths("/bodyset/b1");
        imuDataReporter2.set_compute_accelerations_without_forces(true);
        analyzeIMU.updAnalysisSet().cloneAndAppend(imuDataReporter2);
        analyzeIMU.print("analyzeReportIMUDataNoForces.xml");
        AnalyzeTool roundTrip("analyzeReportIMUDataNoForces.xml");
        roundTrip.run();

        // Load the accelerations from AnalyzeTool from file.
        TimeSeriesTableVec3 accelSignalsNoForces(
                "testIMUDataReporter_no_forces_linear_accelerations.sto");

        TimeSeriesTable accelSignalsNoForcesFlat =
            accelSignalsNoForces.flatten();
        GCVSplineSet accelSplines(accelSignalsNoForcesFlat);
        auto time = accelSignals.getIndependentColumn();
        TimeSeriesTable accelSignalsNoForcesFlatResampled(time);
        for (const auto& label : accelSignalsNoForcesFlat.getColumnLabels()) {
            SimTK::Vector col((int)time.size(), 0.0);
            const auto& thisSpline = accelSplines.get(label);
            for (int i = 0; i < (int)time.size(); ++i) {
                SimTK::Vector timeVec(1, time[i]);
                col[i] = thisSpline.calcValue(timeVec);
            }
            accelSignalsNoForcesFlatResampled.appendColumn(label, col);
        }
        accelSignalsNoForcesFlatResampled.addTableMetaData<std::string>(
                "inDegrees", "no");

        // Compare the original accelerations to the accelerations computed with
        // AnalyzeTool.
        auto accelSignalFlat = accelSignals.flatten();
        auto accelBlock = accelSignalFlat.getMatrixBlock(0, 0,
            accelSignalFlat.getNumRows(),
            accelSignalFlat.getNumColumns());
        auto accelBlockNoForces =
            accelSignalsNoForcesFlatResampled.getMatrixBlock(0, 0,
            accelSignalsNoForcesFlatResampled.getNumRows(),
            accelSignalsNoForcesFlatResampled.getNumColumns());
        auto diff = accelBlock - accelBlockNoForces;
        auto diffSqr = diff.elementwiseMultiply(diff);
        auto sumSquaredError = diffSqr.rowSum();
        // Trapezoidal rule for uniform grid:
        // dt / 2 (f_0 + 2f_1 + 2f_2 + 2f_3 + ... + 2f_{N-1} + f_N)
        double timeInterval = time[(int)time.size()-1] - time[0];
        int numTimes = (int)time.size();
        auto integratedSumSquaredError = timeInterval / 2.0 *
                   (sumSquaredError.sum() +
                    sumSquaredError(1, numTimes - 2).sum());
        SimTK_TEST_EQ_TOL(integratedSumSquaredError, 0.0, 1e-5);
    }
}

TEST_CASE("testMuscleAnalysisSerialization") {
    MuscleAnalysis m;
    m.setComputeMoments(true);
    m.print("manalysis.xml");
    MuscleAnalysis roundTrip("manalysis.xml");
    OPENSIM_ASSERT_ALWAYS(roundTrip.getComputeMoments());
    m.setComputeMoments(false);
    m.print("manalysis.xml");
    // Check deserialization and copying
    roundTrip = MuscleAnalysis("manalysis.xml");
    OPENSIM_ASSERT_ALWAYS(!roundTrip.getComputeMoments());
}

TEST_CASE("AnalyzeTool preserves filtered coordinate count and time range",
        "[filtered-coordinate-sampling]") {
    const bool fromFile = GENERATE(false, true);
    const bool nearDuplicate = GENERATE(false, true);
    const double start = GENERATE(0.0, 678.0);
    CAPTURE(fromFile, nearDuplicate, start);

    Model model = ModelFactory::createPendulum();
    auto* kinematics = new Kinematics(&model);
    kinematics->setInDegrees(false);
    model.addAnalysis(kinematics);
    SimTK::State& state = model.initSystem();

    Storage coordinates;
    Array<string> labels;
    labels.append("time");
    labels.append(model.getCoordinateSet().get(0).getName());
    coordinates.setColumnLabels(labels);
    coordinates.setInDegrees(false);
    for (int i = 0; i <= 100; ++i) {
        const double time = start + 0.01 * i;
        const double value = 0.2 + 0.1 * (time - start);
        coordinates.append(time, 1, &value);
        if (!nearDuplicate && i == 50) {
            const double extraTime = time + 0.0005;
            const double extraValue = 0.2 + 0.1 * (extraTime - start);
            coordinates.append(extraTime, 1, &extraValue);
        }
    }
    if (nearDuplicate) {
        const double time = start + 1.0 + 1e-10;
        const double value = 0.2 + 0.1 * (time - start);
        coordinates.append(time, 1, &value);
    }

    AnalyzeTool analyze(model);
    analyze.setLowpassCutoffFrequency(6.0);
    analyze.setInitialTime(coordinates.getFirstTime());
    analyze.setFinalTime(coordinates.getLastTime());
    analyze.setPrintResultFiles(false);
    if (fromFile) {
        const int precision = IO::GetPrecision();
        IO::SetPrecision(17);
        coordinates.print("testAnalyzeTool_sampling_coordinates.sto");
        IO::SetPrecision(precision);
        analyze.setCoordinatesFileName(
                "testAnalyzeTool_sampling_coordinates.sto");
        analyze.loadStatesFromFile(state);
    } else {
        analyze.setStatesFromMotion(state, coordinates, false);
    }
    REQUIRE(analyze.run());

    const Storage& output = *kinematics->getPositionStorage();
    REQUIRE(output.getSize() == coordinates.getSize());
    const double first = coordinates.getFirstTime();
    const double last = coordinates.getLastTime();
    CHECK_THAT(output.getFirstTime(), Catch::Matchers::WithinAbs(first, 1e-12));
    CHECK_THAT(output.getLastTime(), Catch::Matchers::WithinAbs(last, 1e-12));
    const double dt = (last - first) / (coordinates.getSize() - 1);
    for (int i = 0; i < output.getSize(); ++i) {
        CHECK_THAT(output.getStateVector(i)->getTime(),
                Catch::Matchers::WithinAbs(first + i * dt, 1e-12));
    }
}

TEST_CASE("AnalyzeTool sampling preserves filtered numerical results",
        "[filtered-coordinate-sampling]") {
    using Sampling = AnalyzeTool::FilteredCoordinateSampling;
    const bool uniform = GENERATE(false, true);
    const bool inDegrees = GENERATE(false, true);
    CAPTURE(uniform, inDegrees);
    Model model = ModelFactory::createPendulum();
    auto* kinematics = new Kinematics(&model);
    kinematics->setInDegrees(false);
    model.addAnalysis(kinematics);
    SimTK::State& state = model.initSystem();

    Storage coordinates;
    Array<string> labels;
    labels.append("time");
    labels.append("q0");
    coordinates.setColumnLabels(labels);
    coordinates.setInDegrees(inDegrees);
    for (int i = 0; i <= 400; ++i) {
        const double time = 0.005 * i +
                (!uniform && i > 0 && i < 400 ? 0.001 * (i % 2) : 0);
        // A 2 Hz signal with 120 Hz noise; cutoff = 6 Hz, output rate = 200 Hz.
        double value = 0.2 + 0.1 * sin(4 * SimTK::Pi * time) +
                0.01 * sin(240 * SimTK::Pi * time);
        if (inDegrees) value *= 180 / SimTK::Pi;
        coordinates.append(time, 1, &value);
    }

    AnalyzeTool analyze(model);
    analyze.setLowpassCutoffFrequency(6);
    analyze.setInitialTime(0);
    analyze.setFinalTime(2);
    analyze.setPrintResultFiles(false);
    analyze.setFilteredCoordinateSampling(Sampling::FilterGrid);
    analyze.setStatesFromMotion(state, coordinates, inDegrees);
    const Storage denseStates(analyze.getStatesStorage());
    REQUIRE(analyze.run());
    const Storage denseAcceleration(*kinematics->getAccelerationStorage());

    // This is the pre-change implementation, independent of the policy helper.
    Storage oldCoordinates(coordinates);
    oldCoordinates.pad(oldCoordinates.getSize() / 2);
    oldCoordinates.lowpassIIR(6);
    analyze.setLowpassCutoffFrequency(-1);
    analyze.setStatesFromMotion(state, oldCoordinates, inDegrees);
    const Storage& oldStates = analyze.getStatesStorage();
    REQUIRE(oldStates.getSize() == denseStates.getSize());
    for (int i = 0; i < oldStates.getSize(); ++i) {
        CHECK_THAT(oldStates.getStateVector(i)->getTime(),
                Catch::Matchers::WithinAbs(
                        denseStates.getStateVector(i)->getTime(), 1e-12));
        for (int j = 0; j < oldStates.getSmallestNumberOfStates(); ++j) {
            CHECK_THAT(oldStates.getStateVector(i)->getData()[j],
                    Catch::Matchers::WithinAbs(
                            denseStates.getStateVector(i)->getData()[j], 1e-9));
        }
    }

    analyze.setLowpassCutoffFrequency(6);
    analyze.setFilteredCoordinateSampling(Sampling::UniformInputCount);
    analyze.setStatesFromMotion(state, coordinates, inDegrees);
    REQUIRE(analyze.run());
    const Storage& states = analyze.getStatesStorage();
    const Storage& positions = *kinematics->getPositionStorage();
    const Storage& speeds = *kinematics->getVelocityStorage();
    const Storage& accelerations = *kinematics->getAccelerationStorage();
    const GCVSplineSet referenceSplines(5, &denseStates);
    REQUIRE(positions.getSize() == coordinates.getSize());
    REQUIRE(speeds.getSize() == coordinates.getSize());
    REQUIRE(accelerations.getSize() == coordinates.getSize());
    CHECK(states.getFirstTime() < coordinates.getFirstTime());
    CHECK(states.getLastTime() > coordinates.getLastTime());
    CHECK(!positions.isInDegrees());
    for (int i = 0; i < positions.getSize(); ++i) {
        const double time = positions.getStateVector(i)->getTime();
        Array<double> reference(0.0, 2), actual(0.0, 2);
        // Evaluate the dense reference smoothly at common timestamps; linear
        // interpolation of the reference would add its own slope error.
        const SimTK::Vector argument(1, time);
        reference[0] = referenceSplines.get(0).calcValue(argument);
        reference[1] = referenceSplines.get(1).calcValue(argument);
        states.getDataAtTime(time, 2, actual);
        // Position in radians and speed in radians/s, including the endpoints.
        CHECK_THAT(actual[0], Catch::Matchers::WithinAbs(reference[0], 1e-8));
        CHECK_THAT(actual[1], Catch::Matchers::WithinAbs(reference[1], 2e-3));
        Array<double> acceleration(0.0, 1);
        denseAcceleration.getDataAtTime(time, 1, acceleration);
        CHECK_THAT(accelerations.getStateVector(i)->getData()[0],
                Catch::Matchers::WithinAbs(acceleration[0], 2e-3));
        if (time > 0.25 && time < 1.75) {
            CHECK_THAT(positions.getStateVector(i)->getData()[0],
                    Catch::Matchers::WithinAbs(
                            0.2 + 0.1 * sin(4 * SimTK::Pi * time), 5e-4));
            CHECK_THAT(speeds.getStateVector(i)->getData()[0],
                    Catch::Matchers::WithinAbs(
                            0.4 * SimTK::Pi * cos(4 * SimTK::Pi * time), 5e-3));
        }
    }
}

TEST_CASE("AnalyzeTool sampling policy does not change unfiltered coordinates",
        "[filtered-coordinate-sampling]") {
    using Sampling = AnalyzeTool::FilteredCoordinateSampling;
    const auto sampling = GENERATE(
            Sampling::UniformInputCount, Sampling::FilterGrid);
    Model model = ModelFactory::createPendulum();
    SimTK::State& state = model.initSystem();
    Storage coordinates;
    Array<string> labels;
    labels.append("time");
    labels.append("q0");
    coordinates.setColumnLabels(labels);
    coordinates.setInDegrees(false);
    for (const double time : {0.0, 0.1, 0.11, 0.3, 0.6, 1.0}) {
        const double value = 0.2 + 0.1 * time;
        coordinates.append(time, 1, &value);
    }
    AnalyzeTool analyze(model);
    analyze.setFilteredCoordinateSampling(sampling);
    analyze.setStatesFromMotion(state, coordinates, false);
    const Storage& states = analyze.getStatesStorage();
    REQUIRE(states.getSize() == coordinates.getSize());
    for (int i = 0; i < states.getSize(); ++i) {
        CHECK(states.getStateVector(i)->getTime() ==
                coordinates.getStateVector(i)->getTime());
        CHECK_THAT(states.getStateVector(i)->getData()[0],
                Catch::Matchers::WithinAbs(
                        coordinates.getStateVector(i)->getData()[0], 1e-12));
    }
}

TEST_CASE("AnalyzeTool filter-grid policy skips a NaN cutoff",
        "[filtered-coordinate-sampling][filter-grid-compatibility]") {
    using Sampling = AnalyzeTool::FilteredCoordinateSampling;
    Model model = ModelFactory::createPendulum();
    SimTK::State& state = model.initSystem();
    Storage coordinates;
    Array<string> labels;
    labels.append("time");
    labels.append("q0");
    coordinates.setColumnLabels(labels);
    coordinates.setInDegrees(true);
    for (int i = 0; i <= 100; ++i) {
        const double time = i * 0.01;
        const double value = 10.0 + time;
        coordinates.append(time, 1, &value);
    }

    // The original cutoff >= 0 guard skips filtering for NaN.
    AnalyzeTool analyze(model);
    analyze.setStatesFromMotion(state, coordinates, true);
    const Storage expected(analyze.getStatesStorage());
    REQUIRE(expected.getSize() > 0);

    analyze.setFilteredCoordinateSampling(Sampling::FilterGrid);
    analyze.setLowpassCutoffFrequency(SimTK::NaN);
    REQUIRE_NOTHROW(analyze.setStatesFromMotion(state, coordinates, true));
    const Storage& actual = analyze.getStatesStorage();
    REQUIRE(actual.getSize() == expected.getSize());
    for (int i = 0; i < actual.getSize(); ++i) {
        CHECK(actual.getStateVector(i)->getTime() ==
                expected.getStateVector(i)->getTime());
        for (int j = 0; j < expected.getSmallestNumberOfStates(); ++j) {
            CHECK_THAT(actual.getStateVector(i)->getData()[j],
                    Catch::Matchers::WithinAbs(
                            expected.getStateVector(i)->getData()[j], 1e-12));
        }
    }
    analyze.setFilteredCoordinateSampling(Sampling::UniformInputCount);
    REQUIRE_THROWS_AS(analyze.setStatesFromMotion(state, coordinates, true),
            Exception);
}

TEST_CASE("AnalyzeTool validates short filtered coordinate inputs",
        "[filtered-coordinate-sampling][short-filtered-input]") {
    const int count = GENERATE(2, 3, 4);
    CAPTURE(count);
    Model model = ModelFactory::createPendulum();
    SimTK::State& state = model.initSystem();
    Storage coordinates;
    Array<string> labels;
    labels.append("time");
    labels.append("q0");
    coordinates.setColumnLabels(labels);
    coordinates.setInDegrees(true);
    const double value = 10.0;
    for (int i = 0; i < count; ++i) {
        coordinates.append(i * 0.01, 1, &value);
    }
    AnalyzeTool analyze(model);
    analyze.setLowpassCutoffFrequency(6);
    if (count < 4) {
        REQUIRE_THROWS_WITH(
                analyze.setStatesFromMotion(state, coordinates, true),
                Catch::Matchers::ContainsSubstring("at least four samples"));
    } else {
        REQUIRE_NOTHROW(analyze.setStatesFromMotion(state, coordinates, true));
        CHECK(analyze.getStatesStorage().getSize() >= 6);
    }
}

TEST_CASE("AnalyzeTool sampling policy serialization and validation",
        "[filtered-coordinate-sampling]") {
    using Sampling = AnalyzeTool::FilteredCoordinateSampling;
    AnalyzeTool analyze;
    CHECK(analyze.getFilteredCoordinateSampling() ==
            Sampling::UniformInputCount);
    const auto sampling = GENERATE(
            Sampling::UniformInputCount, Sampling::FilterGrid);
    analyze.setFilteredCoordinateSampling(sampling);
    CHECK(AnalyzeTool(analyze).getFilteredCoordinateSampling() == sampling);
    AnalyzeTool assigned;
    assigned = analyze;
    CHECK(assigned.getFilteredCoordinateSampling() == sampling);
    analyze.print("testAnalyzeTool_sampling.xml");
    AnalyzeTool roundTrip("testAnalyzeTool_sampling.xml", false);
    CHECK(roundTrip.getFilteredCoordinateSampling() == sampling);
    REQUIRE_THROWS_AS(analyze.setFilteredCoordinateSampling(
            static_cast<Sampling>(-1)), Exception);
    analyze.getPropertySet().get("filtered_coordinate_sampling")
            ->setValue(string("unsupported"));
    REQUIRE_THROWS_WITH(analyze.getFilteredCoordinateSampling(),
            Catch::Matchers::ContainsSubstring("uniform_input_count or"));
    analyze.print("testAnalyzeTool_sampling_invalid.xml");
    REQUIRE_THROWS_WITH(
            AnalyzeTool("testAnalyzeTool_sampling_invalid.xml", false),
            Catch::Matchers::ContainsSubstring("filtered_coordinate_sampling"));
}

TEST_CASE("AnalyzeTool sampling respects explicit speeds and states files",
        "[filtered-coordinate-sampling]") {
    Model model = ModelFactory::createPendulum();
    SimTK::State& state = model.initSystem();
    Storage coordinates;
    Array<string> labels;
    labels.append("time");
    labels.append("q0");
    coordinates.setColumnLabels(labels);
    coordinates.setInDegrees(false);
    const double position = 0.2;
    for (int i = 0; i <= 100; ++i) {
        coordinates.append(i * 0.01, 1, &position);
    }
    AnalyzeTool analyze(model);
    analyze.setLowpassCutoffFrequency(6);
    analyze.setStatesFromMotion(state, coordinates, false);
    const Storage expected(analyze.getStatesStorage());

    SECTION("states_file bypasses coordinate filtering") {
        expected.print("testAnalyzeTool_sampling_states.sto");
        analyze.setStatesFileName("testAnalyzeTool_sampling_states.sto");
        // This would be rejected if coordinate filtering were applied.
        analyze.setLowpassCutoffFrequency(1000);
        analyze.loadStatesFromFile(state);
        const Storage& actual = analyze.getStatesStorage();
        REQUIRE(actual.getSize() == expected.getSize());
        for (int i = 0; i < actual.getSize(); ++i) {
            CHECK_THAT(actual.getStateVector(i)->getTime(),
                    Catch::Matchers::WithinAbs(
                            expected.getStateVector(i)->getTime(), 1e-8));
        }
    }
    SECTION("speeds_file still overrides differentiated coordinates") {
        coordinates.print("testAnalyzeTool_sampling_override.sto");
        analyze.setCoordinatesFileName("testAnalyzeTool_sampling_override.sto");
        analyze.loadStatesFromFile(state);
        const Storage fileStates(analyze.getStatesStorage());
        Storage speeds;
        labels[1] = model.getCoordinateSet().get(0).getSpeedName();
        speeds.setColumnLabels(labels);
        speeds.setInDegrees(false);
        const double speed = 0.7;
        for (int i = 0; i < fileStates.getSize(); ++i) {
            speeds.append(fileStates.getStateVector(i)->getTime(), 1, &speed);
        }
        const int precision = IO::GetPrecision();
        IO::SetPrecision(17);
        speeds.print("testAnalyzeTool_sampling_speeds.sto");
        IO::SetPrecision(precision);
        analyze.setSpeedsFileName("testAnalyzeTool_sampling_speeds.sto");
        analyze.loadStatesFromFile(state);
        const Storage& actual = analyze.getStatesStorage();
        REQUIRE(actual.getSize() == fileStates.getSize());
        for (int i = 0; i < actual.getSize(); ++i) {
            CHECK_THAT(actual.getStateVector(i)->getData()[1],
                    Catch::Matchers::WithinAbs(speed, 1e-12));
        }
    }
}

TEST_CASE("AnalyzeTool rejects invalid filtered coordinate sampling",
        "[filtered-coordinate-sampling]") {
    Model model = ModelFactory::createPendulum();
    SimTK::State& state = model.initSystem();
    Storage coordinates;
    Array<string> labels;
    labels.append("time");
    labels.append("q0");
    coordinates.setColumnLabels(labels);
    coordinates.setInDegrees(false);
    const double value = 0.2;
    for (int i = 0; i <= 100; ++i) coordinates.append(i * 0.01, 1, &value);
    AnalyzeTool analyze(model);
    analyze.setLowpassCutoffFrequency(6);
    SECTION("empty") {
        coordinates.purge();
    }
    SECTION("one sample") {
        coordinates.purge();
        coordinates.append(0.0, 1, &value);
    }
    SECTION("non-increasing timestamps") {
        coordinates.append(0.5, 1, &value, false);
    }
    SECTION("non-finite timestamp") {
        coordinates.append(SimTK::NaN, 1, &value, false);
    }
    SECTION("non-finite cutoff") {
        analyze.setLowpassCutoffFrequency(SimTK::NaN);
    }
    SECTION("cutoff at the output Nyquist frequency") {
        analyze.setLowpassCutoffFrequency(50);
    }
    REQUIRE_THROWS_AS(analyze.setStatesFromMotion(state, coordinates, false),
            Exception);
}
