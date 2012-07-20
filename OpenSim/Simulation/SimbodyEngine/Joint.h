#ifndef __Joint_h__
#define __Joint_h__
// Joint.h
// Author: Frank C. Anderson, Peter Loan, Ajay Seth
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
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
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Function.h>
#include "Body.h"
#include "Coordinate.h"
#include <OpenSim/Simulation/Model/CoordinateSet.h>

namespace OpenSim {

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

	virtual void setLocationInParent(const SimTK::Vec3& aLocation);
	virtual void getLocationInParent(SimTK::Vec3& rLocation) const;
	virtual void setOrientationInParent(const SimTK::Vec3& aOrientation);
	virtual void getOrientationInParent(SimTK::Vec3& rOrientation) const;

	// A set of functions that use double[] to be invoked by GUI, not Vec3 aware
	virtual void getOrientationInChild(double rOrientation[]) const {
		const SimTK::Vec3& _orientation = get_orientation();
		for(int i=0; i<3; i++) rOrientation[i]=_orientation[i];
	};
	virtual void getOrientationInParent(double rOrientation[]) const {
		const SimTK::Vec3& _orientationInParent = get_orientation_in_parent();
		for(int i=0; i<3; i++) rOrientation[i]=_orientationInParent[i];
	};
	virtual void getLocationInChild(double rLocation[]) const {
		const SimTK::Vec3& _location = get_location();
		for(int i=0; i<3; i++) rLocation[i]=_location[i];
	};
	virtual void getLocationInParent(double rLocation[]) const {
		const SimTK::Vec3& _locationInParent = get_location_in_parent();
		for(int i=0; i<3; i++) rLocation[i]=_locationInParent[i];
	};

	virtual void setLocationInChild(const SimTK::Vec3& aLocation) {
		set_location(aLocation);
	}

	virtual const SimTK::Vec3& getLocationInChild() const {
		return get_location();
	};

	// Coordinate Set
	const CoordinateSet& getCoordinateSet() const { return get_CoordinateSet(); }

	virtual bool getReverse() const { return get_reverse(); }

	//Model building
	virtual int numCoordinates() const = 0;

	/** Verify that the parent specified by this joint is in the underlying Simbody 
	    model/system. If the parent is not connected (does not have a valid MobilzedBodyIndex) 
		then throw an exception. It is up to the assembly routine to make sure it is
		connecting in a valid sequence - not the joint. */
	virtual void checkParentBody();

	// Utility
	virtual bool isCoordinateUsed(Coordinate& aCoordinate) const;
	
	// Computation
	/** Given some system mobility (generalized) forces, calculate the equivalent spatial body force for this Joint. 
	Keep in mind that there are typically nm < 6 mobilities per joint with an infinte set of solutions that can map
	nm gen forces to 6 spatial force components (3 for torque + 3 for force). The solution returned provides the
	"most" effective force and torque in the joint frame. This means the smallest magnituded force and/or 
	torque that will result in the same generalized force. If a generalized force is defined along/about a joint
	axis, then this should be evident in the reported results as a force or torque on the same axis. 
	NOTE: Joints comprised of multiple mobilizers and/or constraints, should override this method and account for multiple
	      internal components 
	@param state constaining the generalized coordinate and speed values 
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
	SimTK::MobilizedBodyIndex getMobilizedBodyIndex(Body *aBody) const {return aBody->_index;} 
	void setMobilizedBodyIndex(Body *aBody, SimTK::MobilizedBodyIndex index) const {aBody->_index = index;}
	void setCoordinateMobilizedBodyIndex(Coordinate *aCoord, SimTK::MobilizedBodyIndex index) const {aCoord->_bodyIndex = index;}
	void setCoordinateMobilizerQIndex(Coordinate *aCoord, int index) const
		{ aCoord->_mobilizerQIndex = SimTK::MobilizerQIndex(index);}
	void setCoordinateModel(Coordinate *aCoord, Model *aModel) const {aCoord->_model = aModel;}


	/* Calculate the equivalent spatial force, FB_G, acting on a mobilized body specified by index 
	   acting at its mobilizer frame B, expressed in ground.  */
	SimTK::SpatialVec calcEquivalentSpatialForceForMobilizedBody(const SimTK::State &s, const SimTK::MobilizedBodyIndex mbx, const SimTK::Vector &mobilityForces) const;

private:
//=============================================================================
// DATA
//=============================================================================

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

#endif // __Joint_h__


