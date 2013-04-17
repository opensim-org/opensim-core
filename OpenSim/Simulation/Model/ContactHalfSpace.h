#ifndef __ContactHalfSpace_h__
#define __ContactHalfSpace_h__
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ContactHalfSpace.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
// INCLUDE
#include "ContactGeometry.h"

namespace OpenSim {

/**
 * This class represents a half space (that is, everything to one side of an infinite plane)
 * for use in contact modeling.  In its local coordinate system, all points for which x>0 are
 * considered to be inside the geometry.  Its location and orientation properties can be used
 * to move and rotateit to represent other half spaces.
 *
 * @author Peter Eastman
 */
class OSIMSIMULATION_API ContactHalfSpace : public ContactGeometry {
OpenSim_DECLARE_CONCRETE_OBJECT(ContactHalfSpace, ContactGeometry);

public:
//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTION
    /**
     * Construct an empty, uninitialized ContactHalfSpace.
     */
    ContactHalfSpace();
    /**
     * Construct a ContactHalfSpace.  All points in its local coordinate system for which
     * x>0 are considered to be inside the geometry.
     *
	 * @param location     the location of the geometry within the Body it is attached to
	 * @param orientation  the orientation of the half space within the Body it is attached to
     * @param body         the Body this half space is attached to
     */
    ContactHalfSpace(const SimTK::Vec3& location, const SimTK::Vec3& orientation, Body& body);
    /**
     * Construct a ContactHalfSpace.  All points in its local coordinate system for which
     * x>0 are considered to be inside the geometry.
     *
	 * @param location     the location of the geometry within the Body it is attached to
	 * @param orientation  the orientation of the half space within the Body it is attached to
     * @param body         the Body this half space is attached to
     * @param name         the name of this object
     */
    ContactHalfSpace(const SimTK::Vec3& location, const SimTK::Vec3& orientation, Body& body, const std::string& name);
	ContactHalfSpace(const ContactHalfSpace& geom);

    SimTK::ContactGeometry createSimTKContactGeometry();
private:
    // INITIALIZATION
	void setNull();

//=============================================================================
};	// END of class ContactHalfSpace
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ContactHalfSpace_h__
