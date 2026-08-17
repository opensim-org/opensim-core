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
 * A subclass of MarkersReference that provides marker data via a buffer
 * rather than a static reference. Use this class in applications that
 * require reference marker data to be updated from live input streams
 * (e.g., real-time inverse kinematics with InverseKinematicsSolver).
 *
 * Usage:
 * @code
 * auto markersRef = std::make_shared<BufferedMarkersReference>(seedTable,
 *         markerWeights);
 * InverseKinematicsSolver ikSolver(model, markersRef, coordinateRefs);
 * ikSolver.setAdvanceTimeFromReference(true);
 * ikSolver.assemble(state);
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
class OSIMSIMULATION_API BufferedMarkersReference : public MarkersReference {
OpenSim_DECLARE_CONCRETE_OBJECT(BufferedMarkersReference, MarkersReference);
public:
    // CONSTRUCTION
    BufferedMarkersReference();
    BufferedMarkersReference(const BufferedMarkersReference&) = default;
    BufferedMarkersReference(BufferedMarkersReference&&) = default;
    BufferedMarkersReference& operator=(
            const BufferedMarkersReference&) = default;

    // Use MarkersReference convenience constructors (e.g., from a
    // TimeSeriesTable and Set<MarkerWeight>) to seed this reference.
    using MarkersReference::MarkersReference;

    virtual ~BufferedMarkersReference() {}

    // REFERENCE INTERFACE
    /** Get the time range for which this Reference's values are valid,
        based on the loaded marker data. */
    SimTK::Vec2 getValidTimeRange() const override {
        SimTK::Vec2 tableRange = Super::getValidTimeRange();
        return SimTK::Vec2(tableRange[0], SimTK::Infinity);
    };

    /** Get the values from the base MarkersReference's seed data if time
        falls within its range, otherwise pop the next queued values that
        were provided earlier via putValues(). */
    void getValuesAtTime(double time,
            SimTK::Array_<SimTK::Vec3>& values) const override;

    /** Add the passed-in values, and the time at which they were observed,
        to the data queue for later consumption via getNextValuesAndTime(). */
    void putValues(double time,
            const SimTK::RowVector_<SimTK::Vec3>& dataRow);

    double getNextValuesAndTime(
            SimTK::Array_<SimTK::Vec3>& values) override;

    bool hasNext() const override { return !_finished; };

    /** Indicate that no more data will be provided via putValues(), so that
        consumers waiting on this reference (e.g., InverseKinematicsSolver)
        can stop polling once the queued data has been drained. */
    void setFinished(bool finished) {
        _finished = finished;
    };

private:
    // Use a DataQueue for the buffer to decouple producers (e.g., a live
    // marker stream) from the consumer (InverseKinematicsSolver).
    mutable DataQueue_<SimTK::Vec3> _markerDataQueue;
    bool _finished{false};
};  // END of class BufferedMarkersReference

} // namespace OpenSim

#endif // OPENSIM_BUFFERED_MARKERS_REFERENCE_H_
