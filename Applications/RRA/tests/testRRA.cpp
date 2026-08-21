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

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Tools/RRATool.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;
using namespace std;

TEST_CASE("testRRA") {
    RRATool rra("subject01_Setup_RRA.xml");
    rra.run();

    const std::string body = "torso";
    const SimTK::Vec3 standardCOM(0.00598028440188985017, 0.34551, 0.1);
    const Array<double> tolerances(1e-4, 3);

    // compare the adjusted center of mass to OpenSim 1.9.1 values
    Model adjusted_model("subject01_RRA_adjusted.osim");
    const BodySet& bodies = adjusted_model.getBodySet();
    const Body& torso = bodies.get(bodies.getIndex(body));
    SimTK::Vec3 com = torso.getMassCenter();
    CAPTURE(body, com, standardCOM, tolerances);

    for (int i = 0; i < 3; ++i)
        ASSERT_EQUAL(standardCOM[i], com[i], tolerances[i]);

    auto loadsList = adjusted_model.getComponentList<ExternalLoads>();
    OPENSIM_THROW_IF(loadsList.begin() != loadsList.end(), Exception,
        "RRA adjusted model still contains ExternalLoads.");

    auto exfList = adjusted_model.getComponentList<ExternalForce>();
    OPENSIM_THROW_IF(exfList.begin() != exfList.end(), Exception,
        "RRA adjusted model still contains ExternalForce(s).");

    Storage result("ResultsRRA/subject01_walk1_RRA_Kinematics_q.sto"),
            standard("subject01_walk1_RRA_Kinematics_q_standard.sto");
    CHECK_STORAGE_AGAINST_STANDARD(result, standard,
        std::vector<double>(24, 0.5),
        __FILE__, __LINE__, "testRRA: kinematics comparison failed");
}
