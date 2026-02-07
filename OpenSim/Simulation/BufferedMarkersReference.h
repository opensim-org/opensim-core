/* -------------------------------------------------------------------------- *
 *                    OpenSim:  BufferedMarkersReference.h                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2023 Stanford University and the Authors                *
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
#ifndef OPENSIM_BUFFERED_MARKERS_REFERENCE_H_
#define OPENSIM_BUFFERED_MARKERS_REFERENCE_H_

#include "MarkersReference.h"
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Simulation/MarkersReference.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * Subclass of MarkersReference that handles live marker data by providing a buffer
 * that allows clients to push data into and allows the InverseKinematicsSolver to
 * draw data from for solving.
 * This follows the same pattern as BufferedOrientationsReference.
 *
 * @author Selim Gilon
 */

class OSIMSIMULATION_API BufferedMarkersReference
        : public MarkersReference {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            BufferedMarkersReference, MarkersReference);

//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    BufferedMarkersReference();
    BufferedMarkersReference(
            const BufferedMarkersReference&) = default;
    BufferedMarkersReference(BufferedMarkersReference&&) = default;
    BufferedMarkersReference& operator=(
            const BufferedMarkersReference&) = default;

    // Use MarkersReference convenience constructor from TimeSeriesTable
    using MarkersReference::MarkersReference;
    
    // Explicitly declare the constructor that takes TimeSeriesTable and Set<MarkerWeight>
    // This ensures proper SWIG binding on all platforms
    BufferedMarkersReference(const TimeSeriesTable_<SimTK::Vec3>& markerData,
                            const Set<MarkerWeight>& markerWeightSet,
                            Units units = Units(Units::Meters));

    virtual ~BufferedMarkersReference() {}

    //--------------------------------------------------------------------------
    // Reference Interface
    //--------------------------------------------------------------------------
    /** get the time range for which this Reference values are valid,
        based on the loaded marker data.*/
    SimTK::Vec2 getValidTimeRange() const override {
        SimTK::Vec2 tableRange = Super::getValidTimeRange();
        return SimTK::Vec2(tableRange[0], SimTK::Infinity);
    };

    /** get the values from the base MarkersReference, or from
     * the client provided data that was queued earlier using putValues call. */
    void getValuesAtTime(double time,
            SimTK::Array_<SimTK::Vec3>& values) const override;

    /** add passed in values to data processing buffer */
    void putValues(double time, const SimTK::RowVector_<SimTK::Vec3>& dataRow);

    /** get the next values and time from the buffer (for streaming) */
    double getNextValuesAndTime(SimTK::Array_<SimTK::Vec3>& values);

    virtual bool hasNext() const override { 
        return !_finished && _markerBuffer.getNumRows() > 0; 
    };

    void setFinished(bool finished) {
        _finished = finished;
    };

private:
    // Use a TimeSeriesTable for the buffer to support time-based lookup
    mutable TimeSeriesTable_<SimTK::Vec3> _markerBuffer;
    bool _finished{false};
    //=============================================================================
};  // END of class BufferedMarkersReference
//=============================================================================
} // namespace

#endif // OPENSIM_BUFFERED_MARKERS_REFERENCE_H_ 