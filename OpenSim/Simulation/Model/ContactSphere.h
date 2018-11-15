#ifndef OPENSIM_CONTACT_SPHERE_H_
#define OPENSIM_CONTACT_SPHERE_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ContactSphere.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(radius, double,
            "Radius of the sphere (default: 0).");

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
     * @param location     the location of the center of the sphere expressed
     *                     in `frame`.
     * @param frame        the PhysicalFrame this geometry is attached to;
     *                     this constructor connects this ContactSphere to
     *                     the provided `frame`.
     */
    ContactSphere(double radius, const SimTK::Vec3& location,
            const PhysicalFrame& frame);
    /**
     * Construct a ContactSphere.
     *
     * @param radius       the radius of the sphere
     * @param location     the location of the center of the sphere expressed
     *                     in `frame`.
     * @param frame        the PhysicalFrame this geometry is attached to;
     *                     this constructor connects this ContactSphere to
     *                     the provided `frame`.
     * @param name         the name of this object
     */
    ContactSphere(double radius, const SimTK::Vec3& location,
            const PhysicalFrame& frame, const std::string& name);

    SimTK::ContactGeometry createSimTKContactGeometry() const override;

    // ACCESSORS
    /**
     * Get the radius of the sphere.
     */
    double getRadius() const;
    /**
     * %Set the radius of the sphere.
     */
    void setRadius(double radius);

    // VISUALIZATION
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const override;
private:
    // INITIALIZATION
    void setNull();
    void constructProperties();

//=============================================================================
};  // END of class ContactSphere
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_CONTACT_SPHERE_H_ 
