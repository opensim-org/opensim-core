#ifndef OPENSIM_BUFFERED_ORIENTATIONS_REFERENCE_H_
#define OPENSIM_BUFFERED_ORIENTATIONS_REFERENCE_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  BufferedOrientationsReference.h              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
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

#include "OrientationsReference.h"
#include <OpenSim/Common/DataQueue.h>

namespace OpenSim {


//=============================================================================
//=============================================================================
/**
 * Subclass of OrientationsReference that handles live data by providing a DataQueue
 * that allows clients to push data into and allows the InverseKinematicsSolver to 
 * draw data from for solving.
 * Ideally this would be templatized, allowing for all Reference classes to leverage it.
 *
 * @author Ayman Habib
 */

class OSIMSIMULATION_API BufferedOrientationsReference
        : public OrientationsReference {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            BufferedOrientationsReference, OrientationsReference);
 //=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    BufferedOrientationsReference();
    BufferedOrientationsReference(
            const BufferedOrientationsReference&) = default;
    BufferedOrientationsReference(BufferedOrientationsReference&&) = default;
    BufferedOrientationsReference& operator=(
            const BufferedOrientationsReference&) = default;

    // Use OrientationsReference convenience costructor from TimeSeriesTable
    using OrientationsReference::OrientationsReference;

    virtual ~BufferedOrientationsReference() {}

    //--------------------------------------------------------------------------
    // Reference Interface
    //--------------------------------------------------------------------------
    /** get the time range for which this Reference values are valid,
        based on the loaded orientation data.*/
    SimTK::Vec2 getValidTimeRange() const override{
        SimTK::Vec2 tableRange = Super::getValidTimeRange();
        return SimTK::Vec2(tableRange[0], SimTK::Infinity);
    };

    /** get the values from the base OrientationsReference, or from
     * the client provided data that was queued earlier using putValues call. */
    void getValuesAtTime(double time,
            SimTK::Array_<SimTK::Rotation_<double>>& values) const override;

    /** add passed in values to data procesing Queue */
    void putValues(double time, const SimTK::RowVector_<SimTK::Rotation>& dataRow);

    double getNextValuesAndTime(
            SimTK::Array_<SimTK::Rotation_<double>>& values) override;

    virtual bool hasNext() const override { return !_finished; };

    void setFinished(bool finished) { 
        _finished = finished;
    };
private:
    // Use a specialized data structure for holding the orientation data
    mutable DataQueue_<SimTK::Rotation> _orientationDataQueue;
    bool _finished{false};
    //=============================================================================
};  // END of class BufferedOrientationsReference
//=============================================================================
} // namespace

#endif // OPENSIM_BUFFERED_ORIENTATIONS_REFERENCE_H_
