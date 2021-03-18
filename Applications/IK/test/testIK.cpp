/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testIK.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Ajay Seth, Cassidy Kelly, Peter Loan               *
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


// INCLUDES
#include <OpenSim/Common/Storage.h>
#include "OpenSim/Common/STOFileAdapter.h"
#include "OpenSim/Common/TRCFileAdapter.h"
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/OrientationsReference.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Tools/IKTaskSet.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testMarkerWeightAssignments(const std::string& ikSetupFile);
void checkMarkersReferenceConsistencyFromTool(InverseKinematicsTool& ik);

void testInverseKinematicsSolverWithOrientations();
void testInverseKinematicsSolverWithEulerAnglesFromFile();

int main()
{
    int itc = 0;
    SimTK::Array_<std::string> failures;

    try { ++itc;
        testInverseKinematicsSolverWithOrientations(); 
    }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testInverseKinematicsSolverWithOrientations");
    }

    try {
        ++itc;
        testInverseKinematicsSolverWithEulerAnglesFromFile();
    }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testInverseKinematicsSolverWithEulerAnglesFromFile");
    }

    try {
        ++itc;
        testMarkerWeightAssignments("subject01_Setup_InverseKinematics.xml");
    }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testMarkerWeightAssignments");
    }

    Storage  standard("std_subject01_walk1_ik.mot");
    try {
        InverseKinematicsTool ik1("subject01_Setup_InverseKinematics.xml");
        ik1.run();
        Storage result1(ik1.getOutputMotionFileName());
        CHECK_STORAGE_AGAINST_STANDARD(result1, standard, 
            std::vector<double>(24, 0.2), __FILE__, __LINE__, 
            "testInverseKinematicsGait2354 failed");
        cout << "testInverseKinematicsGait2354 passed" << endl;
    }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testInverseKinematicsGait2354");
    }

    try {
        InverseKinematicsTool ik2("subject01_Setup_InverseKinematics_NoModel.xml");
        Model mdl("subject01_simbody.osim");
        mdl.initSystem();
        ik2.setModel(mdl);
        ik2.run();
        Storage result2(ik2.getOutputMotionFileName());
        CHECK_STORAGE_AGAINST_STANDARD(result2, standard, 
            std::vector<double>(24, 0.2), __FILE__, __LINE__, 
            "testInverseKinematicsGait2354 GUI workflow failed");
        cout << "testInverseKinematicsGait2354 GUI workflow passed" << endl;
    }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testInverseKinematicsGait2354_GUI_workflow");
    }

    try {
        InverseKinematicsTool ik3("constraintTest_setup_ik.xml");
        ik3.run();
        cout << "testInverseKinematicsConstraintTest passed" << endl;
    }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testInverseKinematicsConstraintTest");
    }

    try {
        Storage standard("std_subject1_abdbonepin_IK.mot");
        InverseKinematicsTool ik("setup_ik_abdbonepin.xml");
        ik.run();
        Storage result(ik.getOutputMotionFileName());
        // Tolerance of 0.2 degs for rotational coordinates was selected from
        // the other IK regression tests (above) 
        CHECK_STORAGE_AGAINST_STANDARD(result, standard,
                std::vector<double>(17, 0.2), __FILE__, __LINE__,
                "testInverseKinematicsScapulothoracicAbduction failed");
        cout << "testInverseKinematicsScapulothoracicAbduction passed" << endl;
    } catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testInverseKinematicsScapulothoracicAbduction");
    }


    if (!failures.empty()) {
        cout << "Done, with " << failures.size() << " failure(s) out of ";
        cout << itc << " test cases." << endl;
        cout << "Failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done. All cases passed." << endl;
    return 0;
}

void testMarkerWeightAssignments(const std::string& ikSetupFile)
{
    // Check the consistency for the IKTaskSet in the IKTool setup
    InverseKinematicsTool ik(ikSetupFile);

    // Get a copy of the IK tasks
    IKTaskSet tasks = ik.upd_IKTaskSet();

    // assign different weightings so we can verify the assignments
    int nt = tasks.getSize();
    for (int i=0; i < nt; ++i) {
        tasks[i].setWeight(double(i));
    }

    cout << tasks.dump() << endl;
    // update tasks used by the IK tool
    ik.upd_IKTaskSet() = tasks;

    // perform the check
    checkMarkersReferenceConsistencyFromTool(ik);

    // Now reverse the order and reduce the number of tasks
    // so that marker and tasks lists are no longer the same
    IKTaskSet tasks2;
    tasks2.setName("half_markers");

    for (int i = nt-1; i >= 0 ; i -= 2) {
        tasks2.adoptAndAppend(tasks[i].clone());
    }

    cout << tasks2.dump() << endl;
    ik.upd_IKTaskSet() = tasks2;

    // perform the check
    checkMarkersReferenceConsistencyFromTool(ik);

    //Now use more tasks than there are actual markers
    IKMarkerTask* newMarker = new IKMarkerTask();
    newMarker->setName("A");
    newMarker->setWeight(101);
    //insert at the very beginning
    tasks.insert(0, newMarker);
    newMarker = new IKMarkerTask();
    newMarker->setName("B");
    newMarker->setWeight(101);
    // add one in the middle
    tasks.insert(nt / 2, newMarker);
    newMarker = new IKMarkerTask();
    newMarker->setName("C");
    newMarker->setWeight(103);
    // and one more at the end
    tasks.adoptAndAppend(newMarker);
    tasks.setName("Added superfluous tasks");

    cout << tasks.dump() << endl;
    // update the tasks of the IK Tool
    ik.upd_IKTaskSet() = tasks;

    // perform the check: superfluous tasks should also be ignored
    checkMarkersReferenceConsistencyFromTool(ik);
}

void checkMarkersReferenceConsistencyFromTool(InverseKinematicsTool& ik)
{
    MarkersReference markersReference;
    SimTK::Array_<CoordinateReference> coordinateReferences;

    ik.populateReferences(markersReference, coordinateReferences);
    const IKTaskSet& tasks = ik.get_IKTaskSet();

    // Need a model to get a state, doesn't matter which model.
    Model model;
    SimTK::State& state = model.initSystem();

    SimTK::Array_<string> names = markersReference.getNames();
    SimTK::Array_<double> weights;
    markersReference.getWeights(state, weights);

    for (unsigned int i=0; i < names.size(); ++i) {
        std::cout << names[i] << ": " << weights[i];
        int ix = tasks.getIndex(names[i]);
        if (ix > -1) {
            cout << " in TaskSet: " << tasks[ix].getWeight() << endl;
            SimTK_ASSERT_ALWAYS(weights[i] == tasks[ix].getWeight(),
                "Mismatched weight to marker task");
        }
        else {
            OPENSIM_THROW(Exception,
                "Removed IK Task " + names[i] + " still has a Reference.")
        }
    }
}

TimeSeriesTable_<SimTK::Rotation> convertMotionFileToRotations(
    Model& model,
    const std::string& motionFile)
{
    SimTK::State& s0 = model.initSystem();
    TimeSeriesTable anglesTable(motionFile);

    int nt = int(anglesTable.getNumRows());
    const auto& coordNames = anglesTable.getColumnLabels();
    // Vector of times is just the independent column of a TimeSeriesTable
    auto times = anglesTable.getIndependentColumn();

    int dataRate = int(round(double(nt - 1) / (times[nt - 1] - times[0])));
    // get the coordinates of the model
    const auto& coordinates = model.getComponentList<Coordinate>();

    // get bodies as frames that we want to "sense" rotations
    const auto& bodies = model.getComponentList<Body>();

    // Coordinate values in the data file may not correspond to the order
    // of coordinates in the Model. Therefore it is necessary to build a
    // mapping between the data and the model order
    std::vector<int> mapDataToModel;
    // cycle through the coordinates in the model order and store the
    // corresponding column index in the table according to column name
    for (auto& coord : coordinates) {
        int index = -1;
        auto found = std::find(coordNames.begin(), coordNames.end(), coord.getName());
        if (found != coordNames.end())
            index = (int)std::distance(coordNames.begin(), found);
        mapDataToModel.push_back(index);
    }

    cout << "Read in '" << motionFile << "' with " << nt << " rows." << endl;
    cout << "Num coordinates in file: " << coordNames.size() << endl;
    cout << "Num of matched coordinates in model: " << mapDataToModel.size() << endl;

    std::vector<std::string> bodyLabels;
    for (auto& body : bodies) {
        bodyLabels.push_back(body.getName());
    }

    int nc = int(bodyLabels.size());

    // Store orientations as Rotation matrices
    SimTK::Matrix_<SimTK::Rotation> rotations{ int(nt), nc };

    // Apply the read in coordinate values to the model.
    // Then get the rotation of the Bodies in the model and 
    // store them as Rotations and Euler angles in separate tables.
    for (int i = 0; i < nt; ++i) {
        const auto& values = anglesTable.getRowAtIndex(i);
        int cnt = 0;
        for (auto& coord : coordinates) {
            if (mapDataToModel[cnt] >= 0) {
                if (coord.getMotionType() == Coordinate::MotionType::Rotational)
                    coord.setValue(s0,
                        SimTK::convertDegreesToRadians(values(mapDataToModel[cnt++])));
                else
                    coord.setValue(s0, values(mapDataToModel[cnt++]));
            }
        }
        // pose model according to coordinate values and position constraints
        model.realizePosition(s0);

        int j = 0;
        for (auto& body : bodies) {
            // extract the rotation of model bodies w.r.t. ground frame
            const SimTK::Rotation& rot = body.getTransformInGround(s0).R();
            rotations.updElt(i,j++) = rot;
        }
    }

    TimeSeriesTable_<SimTK::Rotation> rotTable{ times, rotations, bodyLabels };
    rotTable.updTableMetaData()
        .setValueForKey("DataRate", std::to_string(dataRate));

    return rotTable;
}

TimeSeriesTableVec3 convertRotationsToEulerAngles(
    const TimeSeriesTable_<SimTK::Rotation>& rotTable)
{
    auto labels = rotTable.getColumnLabels();
    auto& times = rotTable.getIndependentColumn();
    const auto& rotations = rotTable.getMatrix();

    int nc = int(labels.size());
    int nt = int(times.size());

    SimTK::Matrix_<SimTK::Vec3> eulerMatrix(nt, nc, SimTK::Vec3(SimTK::NaN));

    for (int i = 0; i < nt; ++i) {
        for (int j = 0; j < nc; ++j) {
            eulerMatrix.updElt(i, j) = 
                rotations(i, j).convertRotationToBodyFixedXYZ();
        }
    }
    TimeSeriesTableVec3 eulerData{ times, eulerMatrix, labels };
    eulerData.updTableMetaData()
        .setValueForKey("Units", std::string("Radians"));

    return eulerData;
}

void compareMotionTables(const TimeSeriesTable& report,
                         const TimeSeriesTable& standard)
{
    ASSERT(report.getNumRows() == standard.getNumRows());
    ASSERT(report.getNumColumns() == standard.getNumColumns());

    auto reportLabels = report.getColumnLabels();
    auto stdLabels = standard.getColumnLabels();

    std::vector<int> mapStdToReport;
    // cycle through the coordinates in the model order and store the
    // corresponding column index in the table according to column name
    for (auto& label : reportLabels) {
        int index = -1;
        auto found = std::find(stdLabels.begin(), stdLabels.end(), label);
        if (found != stdLabels.end()) {
            // skip any pelvis translations
            if (found->find("pelvis_t") == std::string::npos ||
                    label.length() != 9) {
                index = (int)std::distance(stdLabels.begin(), found);
            }
        }
        mapStdToReport.push_back(index);
    }

    size_t nt = report.getNumRows();

    //For simplicity, we ignore pelvis coordinates 0-5
    for (size_t i = 0; i < mapStdToReport.size(); ++i) {
        if (mapStdToReport[i] >= 0) {
            auto repVec = report.getDependentColumnAtIndex(i);
            auto stdVec = standard.getDependentColumnAtIndex(mapStdToReport[i]);
            auto error = SimTK::Real(SimTK_RTD)*repVec - stdVec;
            cout << "Column '" << reportLabels[i] << "' has RMSE = "
                << sqrt(error.normSqr() / nt) << "degrees" << endl;
            SimTK_ASSERT1_ALWAYS((sqrt(error.normSqr() / nt) < 0.1),
                "Column '%s' FAILED to meet accuracy of 0.1 degree RMS.",
                reportLabels[i].c_str());
        }
    }
}

void testInverseKinematicsSolverWithOrientations()
{
    Model model("subject01_simbody.osim");

    auto orientationsData = convertMotionFileToRotations(
         model, "std_subject01_walk1_ik.mot");

    std::shared_ptr<OrientationsReference> 
        oRefs(new OrientationsReference(orientationsData));
    oRefs->set_default_weight(1.0);

    const std::vector<double>& times = oRefs->getTimes();

    SimTK::Array_<CoordinateReference> coordinateRefs;

    // Add a reporter to get IK computed coordinate values out
    TableReporter* ikReporter = new TableReporter();
    ikReporter->setName("ik_reporter");
    auto coordinates = model.getComponentList<Coordinate>();
    // Hookup reporter inputs to the individual coordinate outputs
    for (auto& coord : coordinates) {
        ikReporter->updInput("inputs").connect(
            coord.getOutput("value"), coord.getName());
    }
    model.addComponent(ikReporter);


    SimTK::State& s0 = model.initSystem();

    // create the solver given the input data
    InverseKinematicsSolver ikSolver(model, nullptr, oRefs, coordinateRefs);
    ikSolver.setAccuracy(1e-4);

    auto timeRange = oRefs->getValidTimeRange();
    cout << "Time range from: " << timeRange[0] << " to " << timeRange[1]
        << "s."<< endl;
    
    s0.updTime() = timeRange[0];
    ikSolver.assemble(s0);

    for (double time : times) {
        s0.updTime() = time;
        cout << "time = " << time << endl;
        ikSolver.track(s0);
        // realize to report to get reporter to pull values from model
        model.realizeReport(s0);
    }

    auto report = ikReporter->getTable();
    const TimeSeriesTable standard("std_subject01_walk1_ik.mot");

    compareMotionTables(report, standard);
}

void testInverseKinematicsSolverWithEulerAnglesFromFile()
{
    Model model("subject01_simbody.osim");
    auto orientationsData = convertMotionFileToRotations(
        model, "std_subject01_walk1_ik.mot");

    auto eulerData = convertRotationsToEulerAngles(orientationsData);
    STOFileAdapter_<SimTK::Vec3>::write(eulerData, "subject1_walk_euler_angles.sto");

    // Add a reporter to get IK computed coordinate values out
    TableReporter* ikReporter = new TableReporter();
    ikReporter->setName("ik_reporter");
    auto coordinates = model.getComponentList<Coordinate>();
    // Hookup reporter inputs to the individual coordinate outputs
    for (auto& coord : coordinates) {
        ikReporter->updInput("inputs").connect(
            coord.getOutput("value"), coord.getName());
    }
    model.addComponent(ikReporter);

    SimTK::State& s0 = model.initSystem();

    std::shared_ptr<OrientationsReference> oRefs(
            new OrientationsReference("subject1_walk_euler_angles.sto"));
    SimTK::Array_<CoordinateReference> coordRefs{};

    // create the solver given the input data
    const double accuracy = 1e-4;
    InverseKinematicsSolver ikSolver(model, nullptr, oRefs, coordRefs);
    ikSolver.setAccuracy(accuracy);

    auto& times = oRefs->getTimes();

    s0.updTime() = times[0];
    ikSolver.assemble(s0);
    //model.getVisualizer().show(s0);

    for (auto time : times) {
        s0.updTime() = time;
        cout << "time = " << time << endl;
        ikSolver.track(s0);
        // realize to report to get reporter to pull values from model
        model.realizeReport(s0);
    }

    auto report = ikReporter->getTable();
    STOFileAdapter::write(report, "ik_euler_tracking_results.sto");

    const TimeSeriesTable standard("std_subject01_walk1_ik.mot");
    compareMotionTables(report, standard);
}
