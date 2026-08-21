/* -------------------------------------------------------------------------- *
 *              OpenSim:  BufferedInverseKinematicsSolver.cpp                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2026 Stanford University and the Authors                *
 * Author(s): Selim Gilon                                                     *
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

#include "BufferedInverseKinematicsSolver.h"

namespace OpenSim {

std::shared_ptr<MarkersReference>
BufferedInverseKinematicsSolver::createSeedReferenceAlias(
        const std::shared_ptr<BufferedMarkersReference>& reference) {
    OPENSIM_THROW_IF(!reference, Exception,
            "BufferedInverseKinematicsSolver requires a non-null "
            "BufferedMarkersReference.");
    return std::shared_ptr<MarkersReference>(
            reference, &reference->updSeedReference());
}

BufferedInverseKinematicsSolver::BufferedInverseKinematicsSolver(
        const Model& model,
        std::shared_ptr<BufferedMarkersReference> markersReference,
        SimTK::Array_<CoordinateReference>& coordinateReferences,
        double constraintWeight)
        : InverseKinematicsSolver(model,
                  createSeedReferenceAlias(markersReference),
                  coordinateReferences, constraintWeight),
          _bufferedMarkersReference(std::move(markersReference)) {
    setAuthors("Ajay Seth, Ayman Habib, Selim Gilon");
}

void BufferedInverseKinematicsSolver::updateGoals(SimTK::State& state) {
    if (!getAdvanceTimeFromReference()) {
        InverseKinematicsSolver::updateGoals(state);
        return;
    }

    OPENSIM_THROW_IF(!_bufferedMarkersReference->hasNext(), Exception,
            "BufferedInverseKinematicsSolver cannot track because the marker "
            "stream is finished and its queue is empty.");

    SimTK::Array_<SimTK::Vec3> markerValues;
    const double nextTime =
            _bufferedMarkersReference->getNextValuesAndTime(markerValues);
    state.setTime(nextTime);
    moveAllMarkerObservations(markerValues);
    AssemblySolver::updateGoals(state);
}

} // namespace OpenSim
