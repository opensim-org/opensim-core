/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testID.cpp                            *
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
#include <string>
#include <iostream>
#include <OpenSim/version.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Simulation/SimbodyEngine/BallJoint.h>
#include <OpenSim/Simulation/SimulationUtilities.h>
#include <OpenSim/Common/GCVSplineSet.h>

using namespace OpenSim;
using namespace std;

void testThoracoscapularShoulderModel();
void testBallJoint();

int main()
{
    try {
        InverseDynamicsTool id1("arm26_Setup_InverseDynamics.xml");
        id1.run();
        Storage result1("Results/arm26_InverseDynamics.sto"), standard1("std_arm26_InverseDynamics.sto");
        CHECK_STORAGE_AGAINST_STANDARD( result1, standard1, 
            std::vector<double>(23, 1e-2), __FILE__, __LINE__,
            "testArm failed");
        cout << "testArm passed" << endl;
        // setOutputGenForceFileName including folder name, test 
        // that folder will be ignored and file is written to Results
        InverseDynamicsTool id12("arm26_Setup_InverseDynamics.xml");
        id12.setOutputGenForceFileName(
                "unused_folder/arm26_InverseDynamics_rerun.sto");
        id12.run();
        Storage result12("Results/arm26_InverseDynamics_rerun.sto");
        CHECK_STORAGE_AGAINST_STANDARD(result1, result12,
                std::vector<double>(23, 1e-5), __FILE__, __LINE__,
                "testArm ignore path in output file spec. failed");
        cout << "testArm output file spec. passed" << endl;

        InverseDynamicsTool id2("subject01_Setup_InverseDynamics.xml");
        id2.run();
        Storage result2("Results/subject01_InverseDynamics.sto"), standard2("std_subject01_InverseDynamics.sto");
        CHECK_STORAGE_AGAINST_STANDARD(result2, standard2, 
            std::vector<double>(23, 2.0), __FILE__, __LINE__,
            "testGait failed");
        cout << "testGait passed" << endl;

        testThoracoscapularShoulderModel();
        cout << "testThoracoscapularShoulderModel passed" << endl;
        // Commented out testBallJoint due to sporadic crash in Model destructor
        // -Ayman 03/21
        //testBallJoint();
        //cout << "testBallJoint passed" << endl;
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

void testThoracoscapularShoulderModel() {
    // Perform ID by setting coordinate values and setting up an
    // InverseDynamicsSovler directly
    Model model("ThoracoscapularShoulderModel.osim");
    SimTK::State& s = model.initSystem();
    TimeSeriesTable motionTable("ThorascapularShoulderModel_static.mot");
    auto labels = motionTable.getColumnLabels();
    
    for (size_t i = 0; i < labels.size(); ++i) {
        const Coordinate& thisCoord = model.getCoordinateSet().get(labels[i]);
        auto thisValue = motionTable.getDependentColumn(labels[i])[0];
        log_cout("ThoracoscapularShoulderModel set {} to {}", labels[i], thisValue);
        thisCoord.setLocked(s, false);
        thisCoord.setValue(s, thisValue, false);
        thisCoord.setSpeedValue(s, 0);
    }
    // IDTool and IDSolver specifically do not perform assembly since it 
    // updates the state directly, so don't assemble here either
    //model.assemble(s);

    InverseDynamicsSolver idSolver(model);
    Set<Muscle>& muscles = model.updMuscles();
    for (int i = 0; i < muscles.getSize(); i++) {
        muscles[i].setAppliesForce(s, false);
    }

    SimTK::Vector idSolverVec = idSolver.solve(s);
    auto coordsInMultibodyOrder = model.getCoordinatesInMultibodyTreeOrder();
    int nCoords = (int)coordsInMultibodyOrder.size();

    // Do the same thing with InverseDynamicsTool
    InverseDynamicsTool idTool("ThorascapularShoulderModel_ID_static.xml");
    idTool.run();
    TimeSeriesTable idToolTable("ThorascapularShoulderModel_ID_output.sto");
    auto idToolLabels = idToolTable.getColumnLabels();
    auto row = idToolTable.getRowAtIndex(0);
    SimTK::Vector idToolVec(nCoords);

    // Reorder Storage file results to multibody tree order just in case
    for (int i = 0; i < nCoords; ++i) {
        std::string genForce = coordsInMultibodyOrder[i]->getName();
        genForce += (coordsInMultibodyOrder[i]->getMotionType() ==
                     Coordinate::Rotational) ? "_moment" : "_force";

        int colInd = (int)idToolTable.getColumnIndex(genForce);
        idToolVec[i] = row[colInd];
    }

    // Compare results
    ASSERT_EQUAL(idSolverVec, idToolVec, 1e-6, __FILE__, __LINE__, 
        "testThoracoscapularShoulderModel failed");
}

void testBallJoint() {
    Model mdl;
    Body* bdy = new Body("body", 1.0, SimTK::Vec3(0), SimTK::Inertia(1));
    BallJoint* jnt = new BallJoint("joint", mdl.getGround(), *bdy);
    mdl.addBody(bdy);
    mdl.addJoint(jnt);
    jnt->upd_coordinates(0).setName("c0");
    jnt->upd_coordinates(1).setName("c1");
    jnt->upd_coordinates(2).setName("c2");

    SimTK::State& s = mdl.initSystem();
    
    // only one joint, coord order is c0, c1, c2
    auto coords = mdl.getCoordinatesInMultibodyTreeOrder();

    // BallJoint is backed by a quaternion.
    // q has length of 4: c0, c1, c2, (unused)
    // u has length of 3: c0, c1, c2
    // state has length 7: [q, u]
    auto coordMap = createSystemYIndexMap(mdl);
    for (const auto& c : coordMap) {
        log_cout("Map entry {} to index {}", c.first,  c.second);
    }

    Array<string> coordStorageLabels;
    coordStorageLabels.append("time");
    for (const auto& c : coords) { 
        coordStorageLabels.append(c->getName()); // order is c0, c1, c2
        log_cout("{}", c->getName());
    }

    Storage coordStorage;
    coordStorage.setColumnLabels(coordStorageLabels);
    for (int i = 0; i < 10; ++i) {
        double vals[3] = {0.1*pow(2, i), 0.2*pow(2, i), 0.3*pow(2, i)};
        SimTK::Vector q(3, vals);
        coordStorage.append(0.1*i, q);
    }
    coordStorage.print("testBallJoint.sto");
    Array<double> timeVec;
    coordStorage.getTimeColumn(timeVec);
    log_cout("{}", timeVec);

    // Setup IDSolver 
    double analyzeTime = 0.5;
    SimTK::Vector udot(mdl.getNumCoordinates());
    GCVSplineSet coordSplines(5, &coordStorage);
    for (int i = 0; i < mdl.getNumCoordinates(); ++i) {
        jnt->upd_coordinates(i).setValue(
                s, coordSplines.evaluate(i, 0, analyzeTime));
        jnt->upd_coordinates(i).setSpeedValue(
                s, coordSplines.evaluate(i, 1, analyzeTime));
        udot[i] = coordSplines.evaluate(i, 2, analyzeTime);
    }
    log_cout("{}", s.getQ());
    log_cout("{}", s.getU());
    log_cout("{}", udot);

    InverseDynamicsSolver idSolver(mdl);
    Set<Muscle>& muscles = mdl.updMuscles();
    for (int i = 0; i < muscles.getSize(); i++) {
        muscles[i].setAppliesForce(s, false);
    }
    SimTK::Vector idSolverVecZeroUDot = idSolver.solve(s);
    SimTK::Vector idSolverVec = idSolver.solve(s, udot);

    // Compare with IDTool
    InverseDynamicsTool idTool;
    string outputFile = "testBallJoint_ID_output.sto";
    idTool.setModel(mdl);
    idTool.setCoordinateValues(coordStorage);
    idTool.setOutputGenForceFileName(outputFile);
    idTool.run();

    // Compare results (should be in same order already)
    Storage idToolOutput(outputFile);
    Array<double> idToolArray;
    idToolOutput.getDataAtTime(analyzeTime, mdl.getNumCoordinates(), idToolArray);
    SimTK::Vector idToolVec(mdl.getNumCoordinates());
    for (int i = 0; i < mdl.getNumCoordinates(); ++i) {
        idToolVec[i] = idToolArray[i];
    }

    // Test should pass when correct udot is used
    ASSERT_EQUAL(idSolverVec, idToolVec, 1e-6, __FILE__, __LINE__,
            "testThoracoscapularShoulderModel failed");

    // Test should not pass when default udot = 0 is used instead
    ASSERT_THROW(Exception,
            ASSERT_EQUAL(idSolverVecZeroUDot, idToolVec, 1e-6, 
            __FILE__, __LINE__, "testThoracoscapularShoulderModel failed"));
}