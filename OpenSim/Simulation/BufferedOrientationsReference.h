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
 * Reference values for the Orientations of model frames that will be used to
 * to compute tracking errors. An Orientation is specified by a Rotation
 * matrix describing the frame orientation with respect to Ground. The 
 * reference also contains weightings that identifies the relative importance
 * of achieving one orientation's reference value over another.
 *
 * @author Ayman Habib
 */
class OSIMSIMULATION_API BufferedOrientationsReference : public OrientationsReference {
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
    BufferedOrientationsReference(const OrientationsReference& orientationsRef);

    virtual ~BufferedOrientationsReference() {}

    //--------------------------------------------------------------------------
    // Reference Interface
    //--------------------------------------------------------------------------
    /** get the time range for which the OrientationsReference values are valid,
        based on the loaded orientation data.*/
    SimTK::Vec2 getValidTimeRange() const override{
        return SimTK::Vec2(-SimTK::Infinity, SimTK::Infinity);
    };

    /** get the value of the OrientationsReference */
    void getValues(const SimTK::State& s,
            SimTK::Array_<SimTK::Rotation_<double>>& values) const override;

    /** add passed in values to OrientationReference */
    void putValues(double time, const SimTK::RowVector_<SimTK::Rotation>& dataRow);

private:
    // Use a specialized data structure for holding the orientation data
    mutable DataQueue_<SimTK::Rotation> _orientationDataQueue;

//=============================================================================
};  // END of class OrientationsReference
//=============================================================================
} // namespace

#endif // OPENSIM_BUFFERED_ORIENTATIONS_REFERENCE_H_
