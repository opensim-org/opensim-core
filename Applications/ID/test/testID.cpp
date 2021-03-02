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

using namespace OpenSim;
using namespace std;

void testThoracoscapularShoulderModel();

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

        InverseDynamicsTool id2("subject01_Setup_InverseDynamics.xml");
        id2.run();
        Storage result2("Results/subject01_InverseDynamics.sto"), standard2("std_subject01_InverseDynamics.sto");
        CHECK_STORAGE_AGAINST_STANDARD(result2, standard2, 
            std::vector<double>(23, 2.0), __FILE__, __LINE__,
            "testGait failed");
        cout << "testGait passed" << endl;

        testThoracoscapularShoulderModel();
        cout << "testThoracoscapularShoulderModel passed" << endl;
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
    
    for (int i = 0; i < labels.size(); ++i) {
        std::cout << labels[i] << " ";
        const Coordinate& thisCoord = model.getCoordinateSet().get(labels[i]);
        auto thisValue = motionTable.getDependentColumn(labels[i])[0];
        std::cout << thisValue << std::endl;
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
    std::cout << idSolverVec << std::endl;
    auto coordsInMultibodyOrder = model.getCoordinatesInMultibodyTreeOrder();

    // Do the same thing with InverseDynamicsTool
    InverseDynamicsTool idTool("ThorascapularShoulderModel_ID_static.xml");
    idTool.run();
    TimeSeriesTable idToolTable("ThorascapularShoulderModel_ID_output.sto");
    auto idToolLabels = idToolTable.getColumnLabels();
    auto row = idToolTable.getRowAtIndex(0);
    SimTK::Vector idToolVec((int)coordsInMultibodyOrder.size());

    // Reorder Storage file results to multibody tree order just in case
    for (int i = 0; i < coordsInMultibodyOrder.size(); ++i) {
        std::string genForce = coordsInMultibodyOrder[i]->getName();
        if (coordsInMultibodyOrder[i]->getMotionType() ==
            Coordinate::Rotational) {
            genForce += "_moment";
        } 
        else if (coordsInMultibodyOrder[i]->getMotionType() ==
                    Coordinate::Translational) {
            genForce += "_force";
        }
        int colInd = (int)idToolTable.getColumnIndex(genForce);
        idToolVec[i] = row[colInd];
    }

    // Compare results
    ASSERT_EQUAL(idSolverVec, idToolVec, 1e-6, __FILE__, __LINE__, 
        "testThoracoscapularShoulderModel failed");
}
