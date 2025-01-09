#ifndef OPENSIM_COORDINATE_SET_H_
#define OPENSIM_COORDINATE_SET_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  CoordinateSet.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan, Ajay Seth                                           *
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

#include <OpenSim/Common/Set.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

namespace OpenSim {
//=============================================================================
//=============================================================================
/**
 * A class for holding a set of coordinates.
 *
 * @authors Peter Loan, Ajay Seth
 */
class OSIMSIMULATION_API CoordinateSet : public Set<Coordinate> {
OpenSim_DECLARE_CONCRETE_OBJECT(CoordinateSet, Set<Coordinate>);

public:
    /** Use Super's constructors. @see Set */
    using Super::Super;

    /**
     * Populate this %Set as a flat list of all Model Coordinates given 
     * the a Model with Joints, which owns the Coordinates.
     */
    void populate(Model& model);

    void getSpeedNames(OpenSim::Array<std::string> &rNames) const;

//=============================================================================
};  // END of class CoordinateSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_COORDINATE_SET_H_
