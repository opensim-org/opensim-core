/* -------------------------------------------------------------------------- *
 *                           OpenSim:  testCMCGait10dof18musc.cpp             *
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
#include <OpenSim/Tools/CMCTool.h>

#define CATCH_CONFIG_MAIN
#include <OpenSim/Auxiliary/catch/catch.hpp>

using namespace OpenSim;

TEST_CASE("testGait10dof18musc") {
    CMCTool cmc("gait10dof18musc_Setup_CMC.xml");
    cmc.run();

    const TimeSeriesTable results(
        "gait10dof18musc_ResultsCMC/walk_subject_states.sto");
    const TimeSeriesTable std(
        "gait10dof18musc_std_walk_subject_states.sto");
    for (const auto& label : results.getColumnLabels()) {
        REQUIRE(SimTK::Test::numericallyEqual(results.getDependentColumn(label),
            std.getDependentColumn(label), 1, 1e-9));
    }
}
