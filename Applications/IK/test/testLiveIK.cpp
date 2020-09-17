/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testLiveIK.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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
#include <OpenSim/Common/TableUtilities.h>
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/OrientationsReference.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Tools/IKTaskSet.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <thread> 

using namespace OpenSim;
using namespace std;

void testInverseKinematicsSolverWithOrientations();
void testInverseKinematicsSolverWithEulerAnglesFromFile();
TimeSeriesTable_<SimTK::Rotation> convertMotionFileToRotations(
        Model& model, const std::string& motionFile);


void compareMotionTables(
        const TimeSeriesTable& report, const TimeSeriesTable& standard) {
    ASSERT(report.getNumRows() == standard.getNumRows());
    ASSERT(report.getNumColumns() == standard.getNumColumns());

    auto reportLabels = report.getColumnLabels();
    auto stdLabels = standard.getColumnLabels();

    std::vector<int> mapStdToReport;
    // cycle through the coordinates in the model order and store the
    // corresponding column index in the table according to column name
    for (const auto& label : reportLabels) {
        int index = -1;
        auto found = std::find(stdLabels.begin(), stdLabels.end(), label);
        if (found != stdLabels.end()) {
            // skip all pelvis translations pelvis_{tx,ty,tz} 
            if (found->find("pelvis_tx") == std::string::npos &&
                    found->find("pelvis_ty") == std::string::npos &&
                    found->find("pelvis_tz") == std::string::npos) {
                index = (int)std::distance(stdLabels.begin(), found);
            }
        }
        mapStdToReport.push_back(index);
    }

    size_t nt = report.getNumRows();

    // For simplicity, we ignore pelvis translations (not in map)
    for (size_t i = 0; i < mapStdToReport.size(); ++i) {
        if (mapStdToReport[i] >= 0) {
            auto repVec = report.getDependentColumnAtIndex(i);
            auto stdVec = standard.getDependentColumnAtIndex(mapStdToReport[i]);
            auto error = SimTK::Real(SimTK_RTD) * repVec - stdVec;
            cout << "Column '" << reportLabels[i]
                 << "' has RMSE = " << sqrt(error.normSqr() / nt) << "degrees"
                 << endl;
            SimTK_ASSERT1_ALWAYS((sqrt(error.normSqr() / nt) < 0.1),
                    "Column '%s' FAILED to meet accuracy of 0.1 degree RMS.",
                    reportLabels[i].c_str());
        }
    }
}

void producer(std::shared_ptr<BufferedOrientationsReference> oRef,
        TimeSeriesTable_<SimTK::Rotation>& dataSource); 

int main() {

    try {
        Model model("subject01_simbody.osim");

        TimeSeriesTable_<SimTK::Rotation> orientationsData =
                convertMotionFileToRotations(
                        model, "std_subject01_walk1_ik.mot");
        TimeSeriesTable_<SimTK::Rotation> trimmedOrientationData{
                orientationsData};
        trimmedOrientationData.trim(0.4, 0.41);
        orientationsData.trimFrom(0.41);
        // Create solver using trimmedOrientationData, then spawn a separate
        // thread to push the remaining data to solver, while main thread
        // waits for input and stops after fixed number of frames/iterations
        std::shared_ptr<BufferedOrientationsReference> oRefs(
                new BufferedOrientationsReference(trimmedOrientationData));
        oRefs->set_default_weight(1.0);

        const std::vector<double>& times = oRefs->getTimes();

        SimTK::Array_<CoordinateReference> coordinateRefs;

        // Add a reporter to get IK computed coordinate values out
        TableReporter* ikReporter = new TableReporter();
        ikReporter->setName("ik_reporter");
        auto coordinates = model.getComponentList<Coordinate>();
        // Hookup reporter inputs to the individual coordinate outputs
        for (const auto& coord : coordinates) {
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
             << "s." << endl;
        //OpenSim::Logger::setLevelString("Debug");
        s0.updTime() = timeRange[0];
        ikSolver.assemble(s0);
        model.realizeReport(s0);
        thread thread1(producer, oRefs, std::ref(orientationsData));
        // we can call join to make sure that all data is produced first
        // then processed in order. 
        //thread1.join();
        auto lastTime = orientationsData.getIndependentColumn().back();
        ikSolver.setAdvanceTimeFromReference(true);
        while (oRefs->hasNext() && s0.getTime() < lastTime) {
            ikSolver.track(s0); // This call advances time in s0 
            model.realizeReport(s0);
        }

        auto report = ikReporter->getTable();
        const TimeSeriesTable standard("std_subject01_walk1_ik.mot");
        compareMotionTables(report, standard);
        thread1.detach();
    } 
    catch (const std::exception& e) { 
        cout << e.what() << endl; 
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
    for (const auto& coord : coordinates) {
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
    for (const auto& body : bodies) {
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
        for (const auto& coord : coordinates) {
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

void testInverseKinematicsSolverWithOrientations()
{
    Model model("subject01_simbody.osim");

    auto orientationsData = convertMotionFileToRotations(
         model, "std_subject01_walk1_ik.mot");

    OrientationsReference oRefs(orientationsData);
    oRefs.set_default_weight(1.0);

    const std::vector<double>& times = oRefs.getTimes();

    SimTK::Array_<CoordinateReference> coordinateRefs;

    // Add a reporter to get IK computed coordinate values out
    TableReporter* ikReporter = new TableReporter();
    ikReporter->setName("ik_reporter");
    auto coordinates = model.getComponentList<Coordinate>();
    // Hookup reporter inputs to the individual coordinate outputs
    for (const auto& coord : coordinates) {
        ikReporter->updInput("inputs").connect(
            coord.getOutput("value"), coord.getName());
    }
    model.addComponent(ikReporter);


    SimTK::State& s0 = model.initSystem();

    // create the solver given the input data
    InverseKinematicsSolver ikSolver(model, nullptr,
            make_shared<OrientationsReference>(oRefs), coordinateRefs);
    ikSolver.setAccuracy(1e-4);

    auto timeRange = oRefs.getValidTimeRange();
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

    auto eulerData = TableUtilities::convertRotationsToEulerAngles(orientationsData);
    STOFileAdapter_<SimTK::Vec3>::write(eulerData, "subject1_walk_euler_angles.sto");

    // Add a reporter to get IK computed coordinate values out
    TableReporter* ikReporter = new TableReporter();
    ikReporter->setName("ik_reporter");
    auto coordinates = model.getComponentList<Coordinate>();
    // Hookup reporter inputs to the individual coordinate outputs
    for (const auto& coord : coordinates) {
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
void producer(std::shared_ptr<BufferedOrientationsReference> oRef,
        TimeSeriesTable_<SimTK::Rotation>& dataSource) {
    auto times = dataSource.getIndependentColumn();
    for (auto t : times) { 
        oRef->putValues(t, dataSource.getNearestRow(t));
    }
}
