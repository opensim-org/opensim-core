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
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include "Body.h"
#include "Coordinate.h"

namespace OpenSim {

class CoordinateSet;

//=============================================================================
//=============================================================================
/**
 * A class implementing an OpenSim Joint.
 *
 * @author Frank C. Anderson, Peter Loan, Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API Joint : public ModelComponent  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Name of the parent body to which this joint connects its owner body. */
	PropertyStr _parentNameProp;
	std::string& _parentName;

	/** Location of the joint in the parent body specified in the parent
	reference frame. */
	PropertyDblVec3 _locationInParentProp;
	SimTK::Vec3& _locationInParent;

	/** Orientation of the joint in the parent body specified in the parent
	reference frame.  Euler XYZ body-fixed rotation angles are used to express
	the orientation. */
	PropertyDblVec3 _orientationInParentProp;
	SimTK::Vec3& _orientationInParent;

	/** Location of the joint in the child body specified in the child
	reference frame.  For SIMM models, this vector is always the zero vector
	(i.e., the body reference frame coincides with the joint).  */
	PropertyDblVec3 _locationProp;
	SimTK::Vec3& _location;

	/** Orientation of the joint in the owing body specified in the owning body
	reference frame.  Euler XYZ body-fixed rotation angles are used to express
	the orientation. */
	PropertyDblVec3 _orientationProp;
	SimTK::Vec3& _orientation;

	/** Set holding the generalized coordinates (q's) that parmeterize this joint. */
	PropertyObj _coordinateSetProp;
	CoordinateSet &_coordinateSet;

	/** Whether the joint transform defines parent->child or child->parent. */
	PropertyBool _reverseProp;
	bool& _reverse;

	/** Body to which this joint belongs. */
	Body *_body;

	/** Body to which this body is attached. */
	Body *_parentBody;

//=============================================================================
// METHODS
//=============================================================================
public:
	// DEFAULT CONSTRUCTION
	Joint();
	// Convenience Constructor
	Joint(const std::string &name, Body &parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
		  Body &body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, bool reverse=false);

	Joint(const Joint &aJoint);
	virtual ~Joint();
	virtual void setup(Model& aModel);
#ifndef SWIG
	Joint& operator=(const Joint &aJoint);
#endif
	void copyData(const Joint &aJoint);
	virtual Object* copy() const = 0;

	// GET & SET
	// Relating to the joint's body
	virtual void setBody(OpenSim::Body &aBody);
	virtual OpenSim::Body& getBody() const;
	virtual void setLocation(const SimTK::Vec3& aLocation);
	virtual void getLocation(SimTK::Vec3& rLocation) const;
	virtual void setOrientation(const SimTK::Vec3& aOrientation);
	virtual void getOrientation(SimTK::Vec3& rOrientation) const;

	// Relating to the parent body
	void setParentName(const std::string& aName);
	std::string getParentName() const;
	virtual void setParentBody(OpenSim::Body &aBody);
	virtual OpenSim::Body& getParentBody() const;
	virtual void setLocationInParent(const SimTK::Vec3& aLocation);
	virtual void getLocationInParent(SimTK::Vec3& rLocation) const;
	virtual void setOrientationInParent(const SimTK::Vec3& aOrientation);
	virtual void getOrientationInParent(SimTK::Vec3& rOrientation) const;

	// A set of functions that use double[] to be invoked by GUI, not Vec3 aware
	virtual void getOrientationInChild(double rOrientation[]) const {
		for(int i=0; i<3; i++) rOrientation[i]=_orientation[i];
	};
	virtual void getOrientationInParent(double rOrientation[]) const {
		for(int i=0; i<3; i++) rOrientation[i]=_orientationInParent[i];
	};
	virtual void getLocationInChild(double rLocation[]) const {
		for(int i=0; i<3; i++) rLocation[i]=_location[i];
	};
	virtual void getLocationInParent(double rLocation[]) const {
		for(int i=0; i<3; i++) rLocation[i]=_locationInParent[i];
	};

	virtual void setLocationInChild(const SimTK::Vec3& aLocation) {
		_location = aLocation;
	};
	virtual void getLocationInChild(SimTK::Vec3& rLocation) const {
		rLocation = _location;
	};

	// Coordinate Set
	virtual CoordinateSet& getCoordinateSet() const { return _coordinateSet; }

	virtual bool getReverse() const { return _reverse; }

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

	OPENSIM_DECLARE_DERIVED(Joint, Object);

	void updateName(const std::string &aName);

protected:
	/** Construct coordinates according to the mobilities of the Joint */
	void constructCoordinates();

	// Methods that allow access for Joint subclasses to data members of objects that
	// Joint befriends like Body, Coordinate and SimbodyEngine
	SimTK::MobilizedBodyIndex getMobilizedBodyIndex(Body *aBody) const {return aBody->_index;} 
	void setMobilizedBodyIndex(Body *aBody, SimTK::MobilizedBodyIndex index) const {aBody->_index = index;}
	void setCoordinateMobilizedBodyIndex(Coordinate *aCoord, SimTK::MobilizedBodyIndex index) const {aCoord->_bodyIndex = index;}
	void setCoordinateMobilityIndex(Coordinate *aCoord, int index) const {aCoord->_mobilityIndex = index;}
	void setCoordinateModel(Coordinate *aCoord, Model *aModel) const {aCoord->_model = aModel;}

    void createSystem(SimTK::MultibodySystem& system) const;
    void initState(SimTK::State& s) const;
    void setDefaultsFromState(const SimTK::State& state);
	virtual int getNumStateVariables() const { return 0; };

	/* Calculate the equivalent spatial force, FB_G, acting on a mobilized body specified by index 
	   acting at its mobilizer frame B, expressed in ground.  */
	SimTK::SpatialVec calcEquivalentSpatialForceForMobilizedBody(const SimTK::State &s, const SimTK::MobilizedBodyIndex mbx, const SimTK::Vector &mobilityForces) const;


private:
	void setNull();
	void setupProperties();
    friend class JointSet;

//=============================================================================
};	// END of class Joint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Joint_h__


