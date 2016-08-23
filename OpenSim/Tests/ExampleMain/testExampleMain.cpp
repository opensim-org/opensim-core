/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testExampleMain.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

void changeVersionNumber(const std::string& filenameOld,
                         const std::string& filenameNew);

int main()
{
    try {
        const std::string result1Filename{"tugOfWar_states.sto"};
        const std::string result1FilenameV2{"tugOfWar_states_v2.sto"};
        changeVersionNumber(result1Filename, result1FilenameV2);
        Storage result1(result1FilenameV2), 
                standard1("std_tugOfWar_states.sto");
        CHECK_STORAGE_AGAINST_STANDARD(result1, standard1, 
                                       Array<double>(0.1, 16), 
                                       __FILE__, 
                                       __LINE__, 
                                       "tugOfWar states failed");
        cout << "tugOfWar states passed\n";

        const std::string result3Filename{"tugOfWar_forces.sto"};
        const std::string result3FilenameV2{"tugOfWar_forces_v2.sto"};
        changeVersionNumber(result3Filename, result3FilenameV2);
        Storage result3(result3FilenameV2), 
                standard3("std_tugOfWar_forces.mot");
        
        Array<double> tols(1.0, 20);
        // 10N is 1% of the muscles maximum isometric force
        tols[0] = tols[1] = 10;

        CHECK_STORAGE_AGAINST_STANDARD(result3, standard3, 
                                       tols, 
                                       __FILE__, 
                                       __LINE__, 
                                       "tugOfWar forces failed");
        cout << "tugOfWar forces passed\n";
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

// Change version number of the file to 1 so that Storage can read it.
void changeVersionNumber(const std::string& filenameOld,
                         const std::string& filenameNew) {
  std::regex versionline{R"([ \t]*version[ \t]*=[ \t]*\d[ \t]*)"};
  std::ifstream fileOld{filenameOld};
  std::ofstream fileNew{filenameNew};
  std::string line{};
  while(std::getline(fileOld, line)) {
    if(std::regex_match(line, versionline))
      fileNew << "version=1\n";
    else
      fileNew << line << "\n";
  }
}
