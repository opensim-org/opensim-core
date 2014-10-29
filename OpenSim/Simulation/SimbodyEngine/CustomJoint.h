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
	CustomJoint(const std::string &name, const Body& parent,
			const SimTK::Vec3& locationInParent, const SimTK::Vec3& orientationInParent,
			const Body& child,
			const SimTK::Vec3& locationInchild, const SimTK::Vec3& orientationInChild,
			SpatialTransform& aSpatialTransform, bool reverse=false);

	// Construct joint with default (empty) coordinates and axes
	CustomJoint(const std::string &name, const Body& parent,
			const SimTK::Vec3& locationInParent, const SimTK::Vec3& orientationInParent,
			const Body& child,
			const SimTK::Vec3& locationInchild, const SimTK::Vec3& orientationInChild,
			bool reverse = false);
	
    // default destructor, copy constructor, copy assignment

	int numCoordinates() const override {return get_CoordinateSet().getSize();};

	// Get and Set Transforms
	const SpatialTransform& getSpatialTransform() const
    {   return get_SpatialTransform(); }
    SpatialTransform& updSpatialTransform() 
    {   return upd_SpatialTransform(); }

	// SCALE
	void scale(const ScaleSet& aScaleSet) override;

	/** Override of the default implementation to account for versioning. */
	void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1)
        override;

private:
    void finalizeFromProperties() override {
        constructCoordinates();
        Super::finalizeFromProperties();
    }
	void connectToModel(Model& aModel) override;
	void doAddToSystem(SimTK::MultibodySystem& system) const override;

	void constructProperties();
    void constructCoordinates();

	template <typename T>
	T createMobilizedBody(SimTK::MobilizedBody& inboard,
		const SimTK::Transform& inboardTransform,
		const SimTK::Body& outboard,
		const SimTK::Transform& outboardTransform,
		int& startingCoorinateIndex) const {};

//==============================================================================
};	// END of class CustomJoint
//==============================================================================
//==============================================================================
template <> 
SimTK::MobilizedBody::FunctionBased
	CustomJoint::createMobilizedBody<SimTK::MobilizedBody::FunctionBased>(
							SimTK::MobilizedBody& inboard,
							const SimTK::Transform& inboardTransform,
							const SimTK::Body& outboard,
							const SimTK::Transform& outboardTransform,
							int& startingCoorinateIndex) const;



} // end of namespace OpenSim

#endif // OPENSIM_CUSTOM_JOINT_H_


