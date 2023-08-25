/* -------------------------------------------------------------------------- *
 *                OpenSim: testMuscleLengthRegression.cpp                     *
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

#include <OpenSim/OpenSim.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <SimTKcommon.h>

#define CATCH_CONFIG_MAIN
#include <OpenSim/Auxiliary/catch.hpp>

namespace {
    using OpenSim::TimeSeriesTable;
    TimeSeriesTable createMuscleLengthsTable(const OpenSim::Model& model,
            const TimeSeriesTable& coordinates) {
        auto statesTraj = OpenSim::StatesTrajectory::createFromStatesTable(
                model, coordinates, true, true, true);
        TimeSeriesTable lengths;
        const auto& muscleSet = model.getMuscles();
        for (const auto& state : statesTraj) {
            model.realizePosition(state);
            SimTK::RowVector lengthsRow(muscleSet.getSize());
            for (int imuscle = 0; imuscle < muscleSet.getSize(); ++imuscle) {
                const auto& muscle = muscleSet.get(imuscle);
                const auto& path = muscle.getPath();
                lengthsRow[imuscle] = path.getLength(state);
            }
            lengths.appendRow(state.getTime(), lengthsRow);
        }
        std::vector<std::string> labels;
        for (int imuscle = 0; imuscle < muscleSet.getSize(); ++imuscle) {
            const auto& muscle = muscleSet.get(imuscle);
            const auto& path = muscle.getAbsolutePathString();
            labels.push_back(path + "|length");
        }
        lengths.setColumnLabels(labels);

        return lengths;
    }
}

using namespace OpenSim;

TEST_CASE("Rajagopal2016, 18 muscles") {
    Model model("subject_walk_armless_18musc.osim");
    model.initSystem();

    TableProcessor tableProcessor =
            TableProcessor("subject_walk_armless_coordinates.mot") |
            TabOpUseAbsoluteStateNames();
    TimeSeriesTable coordinates =
            tableProcessor.processAndConvertToRadians(model);

    TimeSeriesTable lengths = createMuscleLengthsTable(model, coordinates);

    TimeSeriesTable stdLengths(
            "std_testMuscleLengthRegression_subject_walk_armless.sto");
    CHECK(SimTK::Test::numericallyEqual(
            lengths.getMatrix(), stdLengths.getMatrix(),
            static_cast<int>(lengths.getNumColumns()), 1e-6));
}

TEST_CASE("Gait10dof18musc") {
    Model model("walk_gait1018_subject01.osim");
    model.initSystem();

    TableProcessor tableProcessor =
            TableProcessor("walk_gait1018_state_reference.mot") |
            TabOpUseAbsoluteStateNames();
    TimeSeriesTable coordinates =
            tableProcessor.processAndConvertToRadians(model);

    TimeSeriesTable lengths = createMuscleLengthsTable(model, coordinates);

    TimeSeriesTable stdLengths(
            "std_testMuscleLengthRegression_walk_gait1018.sto");
    CHECK(SimTK::Test::numericallyEqual(
            lengths.getMatrix(), stdLengths.getMatrix(),
            static_cast<int>(lengths.getNumColumns()), 1e-6));
}
