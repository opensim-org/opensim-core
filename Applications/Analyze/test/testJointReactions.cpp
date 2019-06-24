/* -------------------------------------------------------------------------- *
 *                      OpenSim:  testJointReactions.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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
#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

int main()
{
    try {
        AnalyzeTool analyze("SinglePin_Setup_JointReaction.xml");
        analyze.run();
        Storage result1("SinglePin_JointReaction_ReactionLoads.sto"),
            standard1("std_SinglePin_JointReaction_ReactionLoads.sto");
        CHECK_STORAGE_AGAINST_STANDARD(result1, standard1, 
            std::vector<double>(standard1.getSmallestNumberOfStates(), 1e-5), __FILE__, __LINE__,
            "SinglePin failed");
        cout << "SinglePin passed" << endl;

        AnalyzeTool analyze2("DoublePendulum3D_Setup_JointReaction.xml");
        analyze2.run();
        Storage result2("DoublePendulum3D_JointReaction_ReactionLoads.sto"),
            standard2("std_DoublePendulum3D_JointReaction_ReactionLoads.sto");
        CHECK_STORAGE_AGAINST_STANDARD(result2, standard2, 
            std::vector<double>(standard2.getSmallestNumberOfStates(), 1e-5), __FILE__, __LINE__,
            "DoublePendulum3D failed");
        cout << "DoublePendulum3D passed" << endl;

        AnalyzeTool analyze3("SinglePin_Setup_JointReaction_FrameKeyword.xml");
        analyze.run();
        Storage result3("SinglePin_JointReaction_ReactionLoads.sto"),
            standard3("std_SinglePin_JointReaction_ReactionLoads_FrameKeyword.sto");
        CHECK_STORAGE_AGAINST_STANDARD(result3, standard3,
            std::vector<double>(standard3.getSmallestNumberOfStates(), 1e-5), __FILE__, __LINE__,
            "SinglePin_FrameKeyword failed");
        cout << "SinglePin_FrameKeyword passed" << endl;

        AnalyzeTool analyze4("DoublePendulum3D_Setup_JointReaction.xml");
        analyze2.run();
        Storage result4("DoublePendulum3D_JointReaction_ReactionLoads.sto"),
            standard4("std_DoublePendulum3D_JointReaction_ReactionLoads_FrameKeyword.sto");
        CHECK_STORAGE_AGAINST_STANDARD(result4, standard4,
            std::vector<double>(standard4.getSmallestNumberOfStates(), 1e-5), __FILE__, __LINE__,
            "DoublePendulum3D_FrameKeyword failed");
        cout << "DoublePendulum3D_FrameKeyword passed" << endl;
    }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
