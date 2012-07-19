#ifndef OPENSIM_CUSTOM_JOINT_H_
#define OPENSIM_CUSTOM_JOINT_H_
// CustomJoint.h
// Author: Frank C. Anderson, Ajay Seth
/*
 * Copyright (c)  2007-12, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


// INCLUDE
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include "Joint.h"
#include <string>

namespace OpenSim {

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
	
	// Construct joint with supplied coordinates and transdorm axes
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

	int numCoordinates() const {return get_CoordinateSet().getSize();};

	// Transforms
	const SpatialTransform& getSpatialTransform() const
    {   return get_SpatialTransform(); }
    SpatialTransform& updSpatialTransform() 
    {   return upd_SpatialTransform(); }

	// SCALE
	void scale(const ScaleSet& aScaleSet);

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


