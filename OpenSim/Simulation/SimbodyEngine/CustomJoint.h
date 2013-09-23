#ifndef OPENSIM_CUSTOM_JOINT_H_
#define OPENSIM_CUSTOM_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  CustomJoint.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
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
#include "Joint.h"

namespace OpenSim {

class SpatialTransform;

//==============================================================================
//                              CUSTOM JOINT
//==============================================================================
/**
 * A class implementing a custom joint.  The underlying component in Simbody
 * is a Function-based mobilizer.
 *
 * @author Frank C. Anderson, Ajay Seth
 */
class OSIMSIMULATION_API CustomJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(CustomJoint, Joint);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with this class. **/
    /**@{**/

	/** Spatial transform defining how the child body moves with respect
	to the parent body as a function of the generalized coordinates.
	Motion over 6 (independent) spatial axes must be defined. */
    OpenSim_DECLARE_UNNAMED_PROPERTY(SpatialTransform,
        "Defines how the child body moves with respect to the parent as "
        "a function of the generalized coordinates.");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
	// CONSTRUCTION
	CustomJoint();
	
	/** Construct joint with supplied coordinates and transform axes */
	CustomJoint(const std::string& name, Body& parent, 
                SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
			    Body& body, SimTK::Vec3 locationInBody, 
                SimTK::Vec3 orientationInBody,
			    SpatialTransform& aSpatialTransform, bool reverse=false);

	// Construct joint with default (empty) coordinates and axes
	CustomJoint(const std::string& name, Body& parent, 
                SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
			    Body& body, SimTK::Vec3 locationInBody, 
                SimTK::Vec3 orientationInBody, bool reverse=false);
	
    // default destructor, copy constructor, copy assignment

	int numCoordinates() const OVERRIDE_11 {return get_CoordinateSet().getSize();};

	// Get and Set Transforms
	const SpatialTransform& getSpatialTransform() const
    {   return get_SpatialTransform(); }
    SpatialTransform& updSpatialTransform() 
    {   return upd_SpatialTransform(); }

	// SCALE
	void scale(const ScaleSet& aScaleSet) OVERRIDE_11;

	/** Override of the default implementation to account for versioning. */
	void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1)
        OVERRIDE_11;

private:
	void connectToModel(Model& aModel) OVERRIDE_11;
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

	void constructProperties();
    void constructCoordinates();

//==============================================================================
};	// END of class CustomJoint
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_CUSTOM_JOINT_H_


