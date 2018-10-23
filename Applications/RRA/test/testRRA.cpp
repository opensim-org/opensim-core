/* -------------------------------------------------------------------------- *
 *                           OpenSim:  testRRA.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Tools/RRATool.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void checkAdjustedModelCOM(string modelFile, string body, 
                const SimTK::Vec3 &standardCOM, 
                const Array<double> &tolerances);

int main() {
    try {
        RRATool rra("subject01_Setup_RRA.xml");
        if (rra.run()){
            checkAdjustedModelCOM( "subject01_RRA_adjusted.osim", "torso",
                      SimTK::Vec3(0.00598028440188985017, 0.34551, 0.1), 
                      Array<double>(1e-4, 3) );
            Storage result("ResultsRRA/subject01_walk1_RRA_Kinematics_q.sto"), 
                    standard("subject01_walk1_RRA_Kinematics_q_standard.sto");
            CHECK_STORAGE_AGAINST_STANDARD(result, standard, 
                std::vector<double>(24, 0.5), 
                __FILE__, __LINE__, "testRRA: kinematics comparison failed");
        }
        else{
            throw(Exception("testRRA FAILED to run to completion."));
        }
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

void checkAdjustedModelCOM(string resultsFile, string body,
    const SimTK::Vec3 &standardCOM, const Array<double> &tolerances)
{
    // compare the adjusted center of mass to OpenSim 1.9.1 values
    Model adjusted_model(resultsFile);
    const BodySet& bodies = adjusted_model.getBodySet();
    const Body& torso = bodies.get(bodies.getIndex(body));
    SimTK::Vec3 com = torso.getMassCenter();
    cout << "body:           " << body << endl;
    cout << "center of mass: (" << com[0] << ", " << com[1] << ", " << com[2] << ")\n";
    cout << "standard COM:   (" << standardCOM[0] << ", " << standardCOM[1] << ", " << standardCOM[2] << ")\n";
    cout << "tolerances:     (" << tolerances[0] << ", " << tolerances[1] << ", " << tolerances[2] << ")\n" << endl;
    for (int i = 0; i < 3; ++i)
        ASSERT_EQUAL(standardCOM[i], com[i], tolerances[i]);

    auto loadsList = adjusted_model.getComponentList<ExternalLoads>();
    OPENSIM_THROW_IF(loadsList.begin() != loadsList.end(), Exception,
        "RRA adjusted model still contains ExternalLoads.");

    auto exfList = adjusted_model.getComponentList<ExternalForce>();
    OPENSIM_THROW_IF(exfList.begin() != exfList.end(), Exception,
        "RRA adjusted model still contains ExternalForce(s).");
}
