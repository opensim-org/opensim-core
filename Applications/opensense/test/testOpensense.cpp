/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testOpensense.cpp                     *
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


// INCLUDES
#include <OpenSim/Simulation/OpenSense/OpenSenseUtilities.h>
#include <OpenSim/Simulation/OpenSense/InverseKinematicsStudy.h>

using namespace OpenSim;
using namespace std;


int main()
{
    int itc = 0;

    // Calibrate model and comapre result to standard
    OpenSenseUtilities::calibrateModelFromOrientations("S07/subject07.osim", "S07/S07_IMU/imuOrientations.sto");
    // Previous line produces a model with same name but "calibrated_" prefix.
    InverseKinematicsStudy ik("setup_IMUtracking.xml");
    // RUN
    ik.run();
    std::cout << "Done. All testOpensense cases passed." << endl;
    return 0;
}
