#ifndef OPENSIM_BUFFERED_MARKERS_REFERENCE_H_
#define OPENSIM_BUFFERED_MARKERS_REFERENCE_H_
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  BufferedMarkersReference.h                    *
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

#include "MarkersReference.h"
#include <OpenSim/Common/DataQueue.h>

namespace OpenSim {

/**
 * Provides marker data via a buffer for use with
 * BufferedInverseKinematicsSolver. This class deliberately does not derive
 * from MarkersReference: buffered and time-based references have separate
 * entry points into inverse kinematics.
 *
 * Usage:
 * @code
 * auto markersRef = std::make_shared<BufferedMarkersReference>(seedTable,
 *         markerWeights);
 * BufferedInverseKinematicsSolver ikSolver(
 *         model, markersRef, coordinateRefs);
 * ikSolver.assemble(state);
 * ikSolver.setAdvanceTimeFromReference(true);
 * // On a producer thread (or interleaved with the code below):
 * //     markersRef->putValues(time, markerDataRow);
 * while (markersRef->hasNext()) {
 *     ikSolver.track(state); // blocks until the next frame is available
 * }
 * @endcode
 *
 * @see BufferedOrientationsReference
 *
 * @author Selim Gilon
 */
class OSIMSIMULATION_API BufferedMarkersReference : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(BufferedMarkersReference, Object);
public:
    // CONSTRUCTION
    BufferedMarkersReference();
    BufferedMarkersReference(const BufferedMarkersReference&) = default;
    BufferedMarkersReference(BufferedMarkersReference&&) = default;
    BufferedMarkersReference& operator=(
            const BufferedMarkersReference&) = default;

    BufferedMarkersReference(
            const TimeSeriesTable_<SimTK::Vec3>& markerData,
            const Set<MarkerWeight>& markerWeightSet,
            Units units = Units(Units::Meters));

    virtual ~BufferedMarkersReference() {}

    /** The time-based reference used to configure and initially assemble the
        inverse kinematics problem. */
    const MarkersReference& getSeedReference() const { return _seedReference; }

    /** Add the passed-in values, and the time at which they were observed,
        to the data queue for later consumption via getNextValuesAndTime(). */
    void putValues(double time,
            const SimTK::RowVector_<SimTK::Vec3>& dataRow);

    double getNextValuesAndTime(
            SimTK::Array_<SimTK::Vec3>& values);

    /** Return true while a producer may add data or queued data remains. */
    bool hasNext() const {
        return !_finished || !_markerDataQueue.isEmpty();
    }

    /** Indicate that no more data will be provided via putValues(), so that
        consumers waiting on this reference can stop polling once the queued
        data has been drained. */
    void setFinished(bool finished) {
        _finished = finished;
    }

private:
    friend class BufferedInverseKinematicsSolver;

    MarkersReference& updSeedReference() { return _seedReference; }

    // Use a DataQueue for the buffer to decouple producers (e.g., a live
    // marker stream) from the consumer (BufferedInverseKinematicsSolver).
    mutable DataQueue_<SimTK::Vec3> _markerDataQueue;
    MarkersReference _seedReference;
    bool _finished{false};
};  // END of class BufferedMarkersReference

} // namespace OpenSim

#endif // OPENSIM_BUFFERED_MARKERS_REFERENCE_H_
