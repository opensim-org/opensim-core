/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testControllerExample.cpp                     *
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
      const std::string result1Filename{"tugOfWar_controls.sto"};
      const std::string result1FilenameV1{"tugOfWar_controls_V1.sto"};
      revertToVersionNumber1(result1Filename, result1FilenameV1);
        Storage result1(result1FilenameV1), 
                standard1("std_tugOfWar_controls.sto");
        CHECK_STORAGE_AGAINST_STANDARD(result1, standard1, 
                                       std::vector<double>(2, 0.01),
                                       __FILE__, 
                                       __LINE__, 
                                       "tugOfWar controls failed");
        cout << "tugOfWar controls passed\n" << endl;

        std::vector<double> tols(16, 0.01);
        // speeds are not matched as precisely
        for(int i =0; i < 6; ++i)
            tols[2*i+1] = 0.03;
        // activations within 2%
        tols[12] = tols[14] = 0.02;

      const std::string result2Filename{"tugOfWar_states.sto"};
      const std::string result2FilenameV1{"tugOfWar_states_V1.sto"};
      revertToVersionNumber1(result2Filename, result2FilenameV1);
        Storage result2(result2FilenameV1), 
                standard2("std_tugOfWar_states.sto");
        CHECK_STORAGE_AGAINST_STANDARD(result2, standard2, 
                                       tols, 
                                       __FILE__, 
                                       __LINE__, 
                                       "tugOfWar states failed");
        cout << "tugOfWar states passed\n" << endl;
    }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
