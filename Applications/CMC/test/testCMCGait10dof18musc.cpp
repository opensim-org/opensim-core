/* -------------------------------------------------------------------------- *
 *                  OpenSim:  testCMCGait10dof18musc.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2023 Stanford University and the Authors                *
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
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/CMCTool.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;

TEST_CASE("testGait10dof18musc") {
    CMCTool cmc("gait10dof18musc_Setup_CMC.xml");
    cmc.run();

    const TimeSeriesTable results(
        "gait10dof18musc_ResultsCMC/walk_subject_states.sto");
    const TimeSeriesTable std(
        "gait10dof18musc_std_walk_subject_states_unix.sto");

    // CMC produces results with differing number of time points on different
    // systems, so we need to interpolate the results to the same time points as
    // the standard results. The shapes of the muscle activity curves also
    // differ slightly, so we allow a larger margin of error for the muscle
    // activity curves. Therefore, this is a weaker test compared to other
    // CMC tests.
    GCVSplineSet resultSplines(results);
    GCVSplineSet stdSplines(std);

    // TODO: Replace with (a new) macro from OpenSim/Moco/Test/Testing.h
    const auto& time = std.getIndependentColumn();
    for (const auto& label : results.getColumnLabels()) {
        const auto& result = resultSplines.get(label);
        const auto& expected = stdSplines.get(label);
        SimTK::Vector timeVec(1, 0.0);
        for (int it = 0; it < static_cast<int>(time.size()); ++it) {
            timeVec[0] = time[it];
            INFO(label << " at time " << time[it] << ": " <<
                result.calcValue(timeVec) << " vs " <<
                expected.calcValue(timeVec));
            REQUIRE((Catch::Approx(result.calcValue(timeVec)).margin(1e-1)
                == expected.calcValue(timeVec)));
        }
    }
}
