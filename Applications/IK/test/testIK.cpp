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
void checkMarkersReferenceConsistencyFromTool(InverseKinematicsTool& ik);

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
    // Check the consistency for the IKTaskSet in the IKTool setup
    InverseKinematicsTool ik(ikSetupFile);

    // Get a copy of the IK tasks
    IKTaskSet tasks = ik.getIKTaskSet();

    // assign different weightings so we can verify the assignments
    int nt = tasks.getSize();
    for (int i=0; i < nt; ++i) {
        tasks[i].setWeight(double(i));
    }

    cout << tasks.dump(true) << endl;
    // update tasks used by the IK tool
    ik.getIKTaskSet() = tasks;

    // perform the check
    checkMarkersReferenceConsistencyFromTool(ik);

    // Now reverse the order and reduce the number of tasks
    // so that marker and tasks lists are no longer the same
    IKTaskSet tasks2;
    tasks2.setName("half_markers");

    for (int i = nt-1; i >= 0 ; i -= 2) {
        tasks2.adoptAndAppend(tasks[i].clone());
    }

    cout << tasks2.dump(true) << endl;
    ik.getIKTaskSet() = tasks2;

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

    cout << tasks.dump(true) << endl;
    // update the tasks of the IK Tool
    ik.getIKTaskSet() = tasks;

    // perform the check: superfluous tasks should be ignored
    checkMarkersReferenceConsistencyFromTool(ik);
}

void checkMarkersReferenceConsistencyFromTool(InverseKinematicsTool& ik)
{
    MarkersReference markersReference;
    SimTK::Array_<CoordinateReference> coordinateReferences;

    ik.populateReferences(markersReference, coordinateReferences);
    const IKTaskSet& tasks = ik.getIKTaskSet();

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
            cout << " default: " << markersReference.get_default_weight() << endl;
            SimTK_ASSERT_ALWAYS(
                weights[i] == markersReference.get_default_weight(),
                "Mismatched weight to default weight");
        }
    }
}
