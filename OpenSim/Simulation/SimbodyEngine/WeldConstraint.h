#ifndef OPENSIM_WELD_CONSTRAINT_H_
#define OPENSIM_WELD_CONSTRAINT_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  WeldConstraint.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
#include "Constraint.h"
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a Weld Constraint.  The underlying Constraint in Simbody
 * is a Constraint::Weld
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API WeldConstraint : public Constraint {
OpenSim_DECLARE_CONCRETE_OBJECT(WeldConstraint, Constraint);

//=============================================================================
// DATA
//=============================================================================
public:

    OpenSim_DECLARE_PROPERTY(location_body_1, SimTK::Vec3,
        "Location of the weld in first body specified in body1 reference frame.");
    OpenSim_DECLARE_PROPERTY(location_body_2, SimTK::Vec3,
        "Location of the weld in second body specified in body2 reference frame.");
    OpenSim_DECLARE_PROPERTY(orientation_body_1, SimTK::Vec3,
        "Orientation of the weld axes on body1 specified in body1's reference frame."  
        "Euler XYZ body-fixed rotation angles are used to express the orientation.");
    OpenSim_DECLARE_PROPERTY(orientation_body_2, SimTK::Vec3,
        "Orientation of the weld axes on body2 specified in body2's reference frame."
        "Euler XYZ body-fixed rotation angles are used to express the orientation.");

//=============================================================================
// METHODS
//=============================================================================
public:
    // CONSTRUCTION
    WeldConstraint();
    // Convenience constructors
    WeldConstraint(const std::string &name, 
        const PhysicalFrame& body1, SimTK::Vec3 locationInBody1, SimTK::Vec3 orientationInBody1,
        const PhysicalFrame& body2, SimTK::Vec3 locationInBody2, SimTK::Vec3 orientationInBody2);
    WeldConstraint(const std::string &name,
        const PhysicalFrame& body1, SimTK::Transform transformInBody1,
        const PhysicalFrame& body2, SimTK::Transform transformInBody2);

    virtual ~WeldConstraint();

    //SET 
    void setBody1ByName(const std::string& aBodyName);
    void setBody1WeldLocation(SimTK::Vec3 location, SimTK::Vec3 orientation=SimTK::Vec3(0));
    void setBody2ByName(const std::string& aBodyName);
    void setBody2WeldLocation(SimTK::Vec3 location, SimTK::Vec3 orientation=SimTK::Vec3(0));

    // Method to set point locations for induced acceleration analysis
    virtual void setContactPointForInducedAccelerations(const SimTK::State &s, SimTK::Vec3 point);

protected:
    /**
    * Extend Component Interface.
    */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    /** Updating XML formating to latest revision */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) override;


private:
    void setNull();
    /** Construct WeldConstraint's properties */
    void constructProperties() override;
    /** Construct WeldConstraint's connectors */
    void constructConnectors() override;
//=============================================================================
};  // END of class WeldConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_WELD_CONSTRAINT_H_


