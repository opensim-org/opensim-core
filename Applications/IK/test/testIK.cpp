/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testIK.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

int main()
{
    try {
        InverseKinematicsTool ik1("subject01_Setup_InverseKinematics.xml");
        ik1.run();
        Storage result1(ik1.getOutputMotionFileName()), standard("std_subject01_walk1_ik.mot");
        CHECK_STORAGE_AGAINST_STANDARD(result1, standard, Array<double>(0.2, 24), __FILE__, __LINE__, "testInverseKinematicsGait2354 failed");
        cout << "testInverseKinematicsGait2354 passed" << endl;

        InverseKinematicsTool ik2("subject01_Setup_InverseKinematics_NoModel.xml");
        Model mdl("subject01_simbody.osim");
        mdl.initSystem();
        ik2.setModel(mdl);
        ik2.run();
        Storage result2(ik2.getOutputMotionFileName());
        CHECK_STORAGE_AGAINST_STANDARD(result2, standard, Array<double>(0.2, 24), __FILE__, __LINE__, "testInverseKinematicsGait2354 GUI workflow failed");
        cout << "testInverseKinematicsGait2354 GUI workflow passed" << endl;
        /*
        InverseKinematicsTool ik3("subjectOld_Setup_InverseKinematics.xml");
        ik3.run();
        Storage result3(ik3.getOutputMotionFileName());
        CHECK_STORAGE_AGAINST_STANDARD(result3, standard, Array<double>(0.2, 24), __FILE__, __LINE__, "testInverseKinematicsGait2354 Old setup failed");
        cout << "testInverseKinematicsGait2354 Old setup passed" << endl;
        */
        InverseKinematicsTool ik4("constraintTest_setup_ik.xml");
        ik4.run();
        cout << "testInverseKinematicsCosntraintTest passed" << endl;
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
