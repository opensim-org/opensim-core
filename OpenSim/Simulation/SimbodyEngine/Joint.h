#ifndef OPENSIM_JOINT_H_
#define OPENSIM_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Joint.h                               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Peter Loan, Ajay Seth                        *
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
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Common/Function.h>

namespace OpenSim {

class Body;
class ScaleSet;

//=============================================================================
//=============================================================================
/**
 * A class implementing an OpenSim Joint.
 *
 * @author Frank C. Anderson, Peter Loan, Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API Joint : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Joint, ModelComponent);

public:

//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with a Joint. **/
    /**@{**/
	OpenSim_DECLARE_PROPERTY(parent_body, std::string, 
		"Name of the parent body to which this joint connects its owner body.");

	OpenSim_DECLARE_PROPERTY(location_in_parent, SimTK::Vec3, 
		"Location of the joint in the parent body specified in the parent "
		"reference frame. Default is (0,0,0).");

	OpenSim_DECLARE_PROPERTY(orientation_in_parent, SimTK::Vec3, 
		"Orientation of the joint in the parent body specified in the parent "
		"reference frame. Euler XYZ body-fixed rotation angles are used to "
		"express the orientation. Default is (0,0,0).");

	OpenSim_DECLARE_PROPERTY(location, SimTK::Vec3, 
		"Location of the joint in the child body specified in the child "
		"reference frame. For SIMM models, this vector is always the zero "
		"vector (i.e., the body reference frame coincides with the joint). ");

	OpenSim_DECLARE_PROPERTY(orientation, SimTK::Vec3, 
		"Orientation of the joint in the owing body specified in the owning body "
		"reference frame.  Euler XYZ body-fixed rotation angles are used to "
		"express the orientation. " );

	OpenSim_DECLARE_UNNAMED_PROPERTY(CoordinateSet, 
		"Set holding the generalized coordinates (q's) that parmeterize this joint." );

    OpenSim_DECLARE_PROPERTY(reverse, bool, 
		"Whether the joint transform defines parent->child or child->parent."); 
    /**@}**/

//=============================================================================
// METHODS
//=============================================================================

	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	/** DEFAULT CONSTRUCTION */
	Joint();

	/** Convenience Constructor */
	Joint(const std::string &name, Body &parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
		  Body &body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, bool reverse=false);

	virtual ~Joint();

	// GET & SET
	/**
	 * Set the body to which this joint belongs.
	 *
	 * @param aBody Body to which this joint should belong.
	 */
	virtual void setBody(OpenSim::Body &aBody);

	/**
	 * Get the body to which this joint belongs.
	 *
	 * @return Body to which this joint belongs.
	 */
	const OpenSim::Body& getBody() const;
	OpenSim::Body& updBody();

	virtual void setLocation(const SimTK::Vec3& aLocation);
	virtual void getLocation(SimTK::Vec3& rLocation) const;
	virtual void setOrientation(const SimTK::Vec3& aOrientation);
	virtual void getOrientation(SimTK::Vec3& rOrientation) const;

	// Relating to the parent body
	void setParentName(const std::string& aName);
	std::string getParentName() const;

	void setParentBody(OpenSim::Body &aBody);
	/**
	 * Get the parent body to which this joint attaches.
	 *
	 * @return Parent body to which this joint attaches.
	 */
	const OpenSim::Body& getParentBody() const;
	OpenSim::Body& updParentBody();

	void setLocationInParent(const SimTK::Vec3& aLocation);
	void getLocationInParent(SimTK::Vec3& rLocation) const;
	void setOrientationInParent(const SimTK::Vec3& aOrientation);
	void getOrientationInParent(SimTK::Vec3& rOrientation) const;

	void setLocationInChild(const SimTK::Vec3& aLocation) {
		set_location(aLocation);
	}

	const SimTK::Vec3& getLocationInChild() const {
		return get_location();
	}

	// Coordinate Set
	const CoordinateSet& getCoordinateSet() const { return get_CoordinateSet(); }

	bool getReverse() const { return get_reverse(); }

	//Model building
	virtual int numCoordinates() const = 0;

	/** Verify that the parent specified by this joint is in the underlying Simbody 
	    model/system. If the parent is not connected (does not have a valid MobilzedBodyIndex) 
		then throw an exception. It is up to the assembly routine to make sure it is
		connecting in a valid sequence - not the joint. */
	void checkParentBody();

	// Utility
	bool isCoordinateUsed(Coordinate& aCoordinate) const;
	
	// Computation
	/** Given some system mobility (generalized) forces, calculate the equivalent spatial body force for this Joint. 
	Keep in mind that there are typically nm < 6 mobilities per joint with an infinte set of solutions that can map
	nm gen forces to 6 spatial force components (3 for torque + 3 for force). The solution returned provides the
	"most" effective force and torque in the joint frame. This means the smallest magnituded force and/or 
	torque that will result in the same generalized force. If a generalized force is defined along/about a joint
	axis, then this should be evident in the reported results as a force or torque on the same axis. 
	NOTE: Joints comprised of multiple mobilizers and/or constraints, should override this method and account for multiple
	      internal components 
	@param s containing the generalized coordinate and speed values 
	@param mobilityForces for the system as computed by inverse dynamics, for example 
	@return spatial force, FB_G, acting on the body connected by this joint at its location B, expressed in ground.  */
	virtual SimTK::SpatialVec calcEquivalentSpatialForce(const SimTK::State &s, const SimTK::Vector &mobilityForces) const;


	/** Joints in general do not contribute power since the reaction space forces
	    are orthogonal to the mobility space. However, when joint motion is prescribed, 
		the internal forces that move the joint will do work. In this case, the power is
		non-zero */
	virtual double calcPower(const SimTK::State &s) const;

	// SCALE
	/**
	* Scale a joint based on XYZ scale factors for the bodies.
	* Generic behavior is to scale the locations on parent and on the body 
	* according to scale factors of the bodies upon which they are located.
	*
	* Joint subclasses should invoke this method before scaling joint specific 
	* properties
	* 
	* @param aScaleSet Set of XYZ scale factors for the bodies.
	*/
	virtual void scale(const ScaleSet& aScaleSet);
    // ModelComponent interface.
	
	/**
	* Perform some set up functions that happen after the
	* object has been deserialized or copied.
	*
	* @param aModel OpenSim model containing this Joint.
	*/
	void connectToModel(Model& aModel) OVERRIDE_11;
protected:

    // TODO: child overrides must invoke Joint::addToSystem()
    // *after* they create the MobilizedBody. This is an API bug
    // since we want to have children invoke parent first.
    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
    void initStateFromProperties(SimTK::State& s) const OVERRIDE_11;
    void setPropertiesFromState(const SimTK::State& state) OVERRIDE_11;

	/** Construct coordinates according to the mobilities of the Joint */
	void constructCoordinates();

	// Methods that allow access for Joint subclasses to data members of objects that
	// Joint befriends like Body, Coordinate and SimbodyEngine
	SimTK::MobilizedBodyIndex getMobilizedBodyIndex(Body *aBody) const; 
	void setMobilizedBodyIndex(Body *aBody, SimTK::MobilizedBodyIndex index) const;
	void setCoordinateMobilizedBodyIndex(Coordinate *aCoord, SimTK::MobilizedBodyIndex index) const {aCoord->_bodyIndex = index;}
	void setCoordinateMobilizerQIndex(Coordinate *aCoord, int index) const
		{ aCoord->_mobilizerQIndex = SimTK::MobilizerQIndex(index);}
	void setCoordinateModel(Coordinate *aCoord, Model *aModel) const {aCoord->_model = aModel;}


	/* Calculate the equivalent spatial force, FB_G, acting on a mobilized body specified by index 
	   acting at its mobilizer frame B, expressed in ground.  */
	SimTK::SpatialVec calcEquivalentSpatialForceForMobilizedBody(const SimTK::State &s, const SimTK::MobilizedBodyIndex mbx, const SimTK::Vector &mobilityForces) const;


//=============================================================================
// DATA
//=============================================================================
protected:
	SimTK::Transform _jointFrameInBody;
	SimTK::Transform _jointFrameInParent;

private:
	/** Body to which this joint belongs. */
	SimTK::ReferencePtr<Body> _body;

	/** Body to which this body is attached. */
	SimTK::ReferencePtr<Body> _parentBody;

	void setNull();
	void constructProperties();
    friend class JointSet;

//=============================================================================
};	// END of class Joint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_JOINT_H_


