/* -------------------------------------------------------------------------- *
 *                  OpenSim:  testCustomActuatorExample.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Cassidy Kelly                                                   *
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

// Author: Cassidy Kelly

//==============================================================================
//==============================================================================

#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

int main()
{
    try {
        const std::string
          result1Filename{"SpringActuatedLeg_states_degrees.sto"};
        const std::string
          result1FilenameV1{"SpringActuatedLeg_states_degrees_V1.sto"};
        revertToVersionNumber1(result1Filename, result1FilenameV1);
        Storage result1(result1FilenameV1),
                standard1("std_SpringActuatedLeg_states_degrees.mot");
        std::vector<double> tolerances(6, 1.0);   // angles have 1 deg tolerance
        tolerances[1] = tolerances[3] = tolerances[5] = 5.0; // angular speeds have a 5 deg/s tolerance

        CHECK_STORAGE_AGAINST_STANDARD(result1, standard1, tolerances, 
            __FILE__, __LINE__, "spring actuated leg states degrees failed");
        log_info("spring actuated leg states degrees passed");
        log_info("");

        std::vector<double> forceTol(2, 1.0); // piston actuator has a tolerance of 1N
        forceTol[1] = 5.0; // spring has a tolerance of 5N 

        const std::string result2Filename{"actuator_forces.sto"};
        const std::string result2FilenameV1{"actuator_forces_V1.sto"};
        revertToVersionNumber1(result2Filename, result2FilenameV1);
        Storage result2(result2FilenameV1),
                standard2("std_actuator_forces.mot");
        CHECK_STORAGE_AGAINST_STANDARD(result2, standard2, forceTol,
                                       __FILE__, __LINE__, "actuator forces failed");
        log_info("actuator forces passed");
        log_info("");
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    log_info("Done");
    return 0;
}
