/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testIK.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
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
#include <string>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Tools/IKTaskSet.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testMarkerWeightAssignments(const std::string& ikSetupFile); 

int main()
{
    try {

        testMarkerWeightAssignments("subject01_Setup_InverseKinematics.xml");

        InverseKinematicsTool ik1("subject01_Setup_InverseKinematics.xml");
        ik1.run();
        Storage result1(ik1.getOutputMotionFileName()), standard("std_subject01_walk1_ik.mot");
        CHECK_STORAGE_AGAINST_STANDARD(result1, standard, 
            std::vector<double>(24, 0.2), __FILE__, __LINE__, 
            "testInverseKinematicsGait2354 failed");
        cout << "testInverseKinematicsGait2354 passed" << endl;

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

        InverseKinematicsTool ik3("constraintTest_setup_ik.xml");
        ik3.run();
        cout << "testInverseKinematicsCosntraintTest passed" << endl;
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

void testMarkerWeightAssignments(const std::string& ikSetupFile)
{
    InverseKinematicsTool ik(ikSetupFile);

    // Get a copy of the IK tasks
    IKTaskSet tasks = ik.getIKTaskSet();

    int nt = tasks.getSize();
    for (int i{ 0 }; i < nt; ++i) {
        tasks[i].setWeight(double(i));
    }

    cout << tasks.dump(true) << endl;
    // update tasks used by the IK tool
    ik.getIKTaskSet() = tasks;

    MarkersReference markersReference;
    SimTK::Array_<CoordinateReference> coordinateReferences;
    
    ik.populateReferences(markersReference, coordinateReferences);

    SimTK::Array_<string> names = markersReference.getNames();

    // Need a model to get a state, doesn't matter which model.
    Model model;
    SimTK::State& state = model.initSystem();

    SimTK::Array_<double> weights;
    markersReference.getWeights(state, weights);

    SimTK_ASSERT_ALWAYS(names.size() == weights.size(),
        "Number of markers does not match number of weights.");

    for (auto i{ 0 }; i < names.size(); ++i) {
        std::cout << names[i] << ": " << weights[i] << " in TaskSet: "
        << tasks.get(names[i]).getWeight() << std::endl;
        SimTK_ASSERT_ALWAYS(weights[i] == tasks.get(names[i]).getWeight(),
            "Mismatched weight to marker");
    }

    IKTaskSet tasks2;
    tasks2.setName("half_markers");

    // Now reverse order and with half the number of tasks
    for (int i{ nt / 2 }; i > -1 ; --i) {
        tasks2.adoptAndAppend(tasks[2*i].clone());
    }

    cout << tasks2.dump(true) << endl;
    ik.getIKTaskSet() = tasks2;

    MarkersReference markersReference2;
    SimTK::Array_<CoordinateReference> coordinateReferences2;

    ik.populateReferences(markersReference2, coordinateReferences2);

    SimTK::Array_<string> names2 = markersReference2.getNames();
    SimTK::Array_<double> weights2;
    markersReference2.getWeights(state, weights2);

    for (auto i{ 0 }; i < names2.size(); ++i) {
        std::cout << names2[i] << ": " << weights2[i];
        int ix = tasks2.getIndex(names2[i]);
        if (ix > -1) {
            cout << " in TaskSet: " << tasks2[ix].getWeight() << endl;
            SimTK_ASSERT_ALWAYS(weights2[i] == tasks2[ix].getWeight(),
                "Mismatched weight to marker task");
        }
        else {
            cout << " default: " << markersReference2.get_default_weight() << endl;
            SimTK_ASSERT_ALWAYS(
                weights2[i] == markersReference2.get_default_weight(),
                "Mismatched weight to default weight");
        }
    }
}


