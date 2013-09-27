#ifndef __ContactSphere_h__
#define __ContactSphere_h__
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ContactSphere.h                          *
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
 * This class represents a spherical object for use in contact modeling.
 *
 * @author Peter Eastman
 */
class OSIMSIMULATION_API ContactSphere : public ContactGeometry {
OpenSim_DECLARE_CONCRETE_OBJECT(ContactSphere, ContactGeometry);

//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyDbl _radiusProp;
	double& _radius;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
    /**
     * Construct an empty, uninitialized ContactSphere.
     */
    ContactSphere();
	/**
	 * Construct a ContactSphere.
	 *
	 * @param radius       the radius of the sphere
	 * @param location     the location of the center of the sphere within the Body it is attached to
	 * @param body         the Body this sphere is attached to
	 */
	ContactSphere(double radius, const SimTK::Vec3& location, Body& body);
	/**
	 * Construct a ContactSphere.
	 *
	 * @param radius       the radius of the sphere
	 * @param location     the location of the center of the sphere within the Body it is attached to
	 * @param body         the Body this sphere is attached to
     * @param name         the name of this object
	 */
    ContactSphere(double radius, const SimTK::Vec3& location, Body& body, const std::string& name);
	ContactSphere(const ContactSphere& geom);

    #ifndef SWIG
    ContactSphere& operator=(const ContactSphere& source) {
        if (&source != this) {
            Super::operator=(source);
            copyData(source);
        }
        return *this;
    }
    #endif

    void copyData(const ContactSphere& source) {
        _radius = source._radius;
    }
    SimTK::ContactGeometry createSimTKContactGeometry();

	// ACCESSORS
	/**
	 * Get the radius of the sphere.
	 */
	double getRadius() const;
	/**
	 * Set the radius of the sphere.
	 */
	void setRadius(double radius);
private:
    // INITIALIZATION
	void setNull();
	void setupProperties();

    // VISUALIZATION
    void generateDecorations(bool fixed, const ModelDisplayHints& hints, 
        const SimTK::State& s, 
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const OVERRIDE_11;

//=============================================================================
};	// END of class ContactSphere
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ContactSphere_h__
