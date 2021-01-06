/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSandbox.cpp                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// This file provides a way to easily prototype or test temporary snippets of
// code during development.

#include <Moco/osimMoco.h>

using namespace OpenSim;

int main() {
    // TODO Logger::setLevel(Logger::Level::Debug);
    MocoTrack track;
    ModelProcessor modelProcessor("DeMers_mod_noarms_welds_4.0.osim");
    modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
    modelProcessor.append(ModOpIgnoreTendonCompliance());
    track.setModel(modelProcessor);
    track.setStatesReference({"r_SLD_mean_coords.sto"});
    track.set_allow_unused_references(true);
    MocoSolution solution = track.solve();
    return EXIT_SUCCESS;
}
