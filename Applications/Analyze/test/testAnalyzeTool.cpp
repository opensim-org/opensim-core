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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Analyses/OutputReporter.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Auxiliary/auxiliaryTestMuscleFunctions.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Analyses/SyntheticIMUDataReporter.h>
#include <OpenSim/Actuators/ModelFactory.h>

using namespace OpenSim;
using namespace std;

void testTutorialOne();
void testActuationAnalysisWithDisabledForce();

// Test different default activations are respected when activation
// states are not provided.
void testTugOfWar(const string& dataFileName, const double& defaultAct);

void testBodyKinematics();

void testIMUDataReporter();

int main()
{
    SimTK::Array_<std::string> failures;

    try { testTutorialOne(); }
    catch (const std::exception& e) {
        cout << e.what() << endl; failures.push_back("testTutorialOne");
    }

    // produce passive force-length curve
    try { testTugOfWar("Tug_of_War_ConstantVelocity.sto", 0.01); }
    catch (const std::exception& e) {
        cout << e.what() << endl; 
        failures.push_back("testTugOfWar CoordinatesOnly: default_act = 0.01");
    }

    // produced passive + active muscle force-length
    try { testTugOfWar("Tug_of_War_ConstantVelocity.sto", 1.0); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testTugOfWar CoordinatesOnly: default_act = 1.0");
    }

    // now supply activation states which should be used instead of the default
    try { testTugOfWar("Tug_of_War_ConstantVelocity_RampActivation.sto", 0.0); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testTugOfWar with activation state provided");
    }

    try { testActuationAnalysisWithDisabledForce(); } 
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testActuationAnalysisWithDisabledForce");
    }
    try { testBodyKinematics(); }
    catch (const std::exception& e) { 
        cout << e.what() << endl;
        failures.push_back("testBodyKinematics");
    }   
    try {
        testIMUDataReporter();
    } catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testIMUDataReporter");
    }   

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;
    return 0;
}

void testTutorialOne() {
    AnalyzeTool analyze1("PlotterTool.xml");
    analyze1.getModel().print("testAnalyzeTutorialOne.osim");
    analyze1.run();
    /* Once this runs to completion we'll make the test more meaningful by comparing output
    * to a validated standard. Let's make sure we don't crash during run first! -Ayman 5/29/12 */
    Storage resultFiberLength("testPlotterTool/BothLegs__FiberLength.sto");
    Storage standardFiberLength("std_BothLegs_fiberLength.sto");
    CHECK_STORAGE_AGAINST_STANDARD(resultFiberLength, standardFiberLength,
        std::vector<double>(100, 0.0001), __FILE__, __LINE__,
        "testAnalyzeTutorialOne failed");
    // const Model& mdl = analyze1.getModel();
    //mdl.updMultibodySystem()
    analyze1.setStatesFileName("plotterGeneratedStatesHip45.sto");
    //analyze1.setModel(mdl);
    analyze1.setName("BothLegsHip45");
    analyze1.run();
    Storage resultFiberLengthHip45("testPlotterTool/BothLegsHip45__FiberLength.sto");
    Storage standardFiberLength45("std_BothLegsHip45__FiberLength.sto");
    CHECK_STORAGE_AGAINST_STANDARD(resultFiberLengthHip45,
        standardFiberLength45, std::vector<double>(100, 0.0001),
        __FILE__, __LINE__, "testAnalyzeTutorialOne at Hip45 failed");
    cout << "testAnalyzeTutorialOne passed" << endl;
}

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
    TimeSeriesTable_<double> results("Analyze_Tug_of_War/Tug_of_War_Millard_Iso_ForceReporter_forces.sto");
    assert(results.getNumColumns() == 1);
    SimTK::Vector forces = results.getDependentColumnAtIndex(0);

    TimeSeriesTable_<double> outputs_table("Analyze_Tug_of_War/Tug_of_War_Millard_Iso_Outputs.sto");
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
            reportTendonAndFiberForcesAcrossFiberLengths(muscle, s);
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
        ASSERT_EQUAL<double>(tf, mf, equilTol);
        // Verify that the current computed and AnalyzeTool reported force are
        // equivalent for the provided motion file
        cout << s.getTime() << " :: muscle-fiber-force: " << mf <<
            " Analyze reported force: " << forces[int(i)] << endl;
        ASSERT_EQUAL<double>(mf, forces[int(i)], equilTol, __FILE__, __LINE__,
            "Total fiber force failed to match reported muscle force.");

        cout << s.getTime() << " :: tendon-force: " << tf <<
            " Analyze Output reported: " << tf_output[int(i)] << endl;
        ASSERT_EQUAL<double>(tf, tf_output[int(i)], equilTol,
            __FILE__, __LINE__,
            "Output reported muscle-tendon force failed to match computed.");

        double delta = (i > 0) ? abs(forces[int(i)]-forces[int(i-1)]) : 0;

        SimTK_ASSERT_ALWAYS(delta < maxDelta,
            "Force trajectory has unexplained discontinuity.");
    }
}

void testActuationAnalysisWithDisabledForce() {
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
    ASSERT_EQUAL(model.getMuscles().getSize() - 1, 
            (int)act_force_table.getNumColumns());
}

void testBodyKinematics() { 
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
    ASSERT_EQUAL<double>(localVelOx.getLast(), speedRot * SimTK_RADIAN_TO_DEGREE, tol);
    ASSERT_EQUAL<double>(localVelOz.getLast(), 0, tol);
    ASSERT_EQUAL<double>(groundVelOx.getLast(), 0, tol);
    ASSERT_EQUAL<double>(groundVelOz.getLast(), speedRot, tol);

    Array<double> groundPosX, groundPosY;
    Storage groundPos("_BodyKinematics_ground_pos_global.sto");
    groundPos.getDataColumn("body_X", groundPosX);
    groundPos.getDataColumn("body_Y", groundPosY);
    ASSERT_EQUAL<double>(groundPosX.getLast(), speedX * duration, tol);
    ASSERT_EQUAL<double>(groundPosY.getLast(), speedY * duration, tol);
}
void testIMUDataReporter() {
    Model pendulum = ModelFactory::createNLinkPendulum(2);

    BodyKinematics* bodyKinematics = new BodyKinematics(&pendulum);
    bodyKinematics->setName("BodyKinematics_fall");
    bodyKinematics->setRecordCenterOfMass(false);
    bodyKinematics->setExpressResultsInLocalFrame(false);
    bodyKinematics->setInDegrees(false);

    SyntheticIMUDataReporter* imuDataReporter =
            new SyntheticIMUDataReporter(&pendulum);
    imuDataReporter->setName("IMU_DataReporter");
    imuDataReporter->reportAllBodies();

    pendulum.addAnalysis(bodyKinematics);
    pendulum.addAnalysis(imuDataReporter);

    SimTK::State& s = pendulum.initSystem();

    Joint& joint = pendulum.updJointSet()[0];
    auto& qi = joint.updCoordinate();
    qi.setValue(s, SimTK::Pi / 2.); // static

    Manager manager(pendulum);
    double duration = 2.0;
    manager.initialize(s);
    s = manager.integrate(duration);

    imuDataReporter->printResults("static", "");
    const TimeSeriesTable_<SimTK::Vec3>& angVelTable =
            imuDataReporter->getAngularVelocitiesTable();
    const TimeSeriesTable_<SimTK::Vec3>& linAccTable =
            imuDataReporter->getLinearAccelerationTable();
    const TimeSeriesTable_<SimTK::Quaternion>& rotationsTable =
            imuDataReporter->getOrientationsTable();
    for (size_t row = 0; row < angVelTable.getNumRows(); ++row) {
        ASSERT_EQUAL<double>(angVelTable.getMatrix()[row][0].norm(), 0., 1e-7);
        ASSERT_EQUAL<double>(angVelTable.getMatrix()[row][1].norm(), 0., 1e-7);
        ASSERT_EQUAL<double>(linAccTable.getMatrix()[row][0].norm(), 0., 1e-7);
        ASSERT_EQUAL<double>(linAccTable.getMatrix()[row][1].norm(), 0., 1e-7);
    }
    // Now allow pendulum to drop under gravity from horizontal
    bodyKinematics->getPositionStorage()->purge();
    qi.setValue(s, 0.); // Horisontal position
    s.setTime(0.);
    Manager manager2(pendulum);
    manager2.initialize(s);
    s = manager2.integrate(duration);
    // Compare results to Body kinematics
    auto orientationTableIMU = imuDataReporter->getOrientationsTable();
    auto orientationTableBodyKin = bodyKinematics->getPositionStorage();
    for (size_t row = 0; row < orientationTableIMU.getNumRows(); ++row) {
        // fromBodyKin has positions followed by rotations for each body
        Array<double>& fromBodyKin =
                orientationTableBodyKin->getStateVector(row)->getData();
        for (int b = 0; b <= 1; b++) {
            SimTK::Vec3 bodyFixedRotations =
                    SimTK::Rotation(orientationTableIMU.getRowAtIndex(row)[b])
                            .convertRotationToBodyFixedXYZ();
            SimTK::Vec3 fromBodyKinRotations = SimTK::Vec3(&fromBodyKin[b * 6 + 3]);
            ASSERT_EQUAL<double>(
                    (bodyFixedRotations - fromBodyKinRotations).norm(), 0., 1e-7);
        }
    }
    /* Attempt to compare to createSyntheticIMUAccelerationSignals  doesn't
    * work now since the latter requires passing in controls!
    *
    std::vector<std::string> framePaths = {"/bodyset/b0", "/bodyset/b1"};
    TimeSeriesTableVec3 accelTableFromUtility =
            createSyntheticIMUAccelerationSignals(
                    pendulum, statesTable, controlsTable, framePaths);
    STOFileAdapter_<SimTK::Vec3>::write(
            accelTableFromUtility, "linacc_fromutils.sto");
            */
}
