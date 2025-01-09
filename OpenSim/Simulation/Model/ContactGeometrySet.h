#ifndef OPENSIM_CONTACT_GEOMETRY_SET_H_
#define OPENSIM_CONTACT_GEOMETRY_SET_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ContactGeometrySet.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2018 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
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

#include "OpenSim/Simulation/Model/ModelComponentSet.h"
#include "ContactGeometry.h"

namespace OpenSim {
//=============================================================================
//=============================================================================
/**
 * A class for holding a set of ContactGeometry components.
 *
 * @authors Peter Eastman
 */
class OSIMSIMULATION_API ContactGeometrySet 
:   public ModelComponentSet<ContactGeometry> {
OpenSim_DECLARE_CONCRETE_OBJECT(ContactGeometrySet, 
                                ModelComponentSet<ContactGeometry>);

public:
    /** Use Super's constructors. @see ModelComponentSet */
    using Super::Super;

    // default copy, assignment operator, and destructor

//=============================================================================
};  // END of class ContactGeometrySet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_CONTACT_GEOMETRY_SET_H_
