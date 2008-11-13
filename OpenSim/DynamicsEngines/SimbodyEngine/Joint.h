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
#include "osimSimbodyEngineDLL.h"
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/Transform.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/AbstractJoint.h>
#include <OpenSim/Simulation/Model/TransformAxisSet.h>
#include "Body.h"
#include "Coordinate.h"

namespace OpenSim {

class CoordinateSet;

//=============================================================================
//=============================================================================
/**
 * A class implementing a Simbody joint.
 *
 * @author Frank C. Anderson, Peter Loan
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API Joint : public AbstractJoint  
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

	/** Simbody ID of the body to which this joint belongs. */
	SimTK::MobilizedBodyIndex _bodyId;

	/** Body to which this joint belongs. */
	Body *_body;

	/** Body to which this body is attached. */
	Body *_parentBody;

	/** Transform for expressing a vector given in the child frame in
	the parent fram. */
	Transform _forwardTransform;

	/** Transform for expressing a vector given in the parent frame in
	the child frame. */
	Transform _inverseTransform;

	/** Simbody engine that contains this joint. */
	//SimbodyEngine* _engine;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	Joint();
	Joint(const Joint &aJoint);
	virtual ~Joint();
	void setup(AbstractDynamicsEngine* aEngine);
	SimbodyEngine* getEngine() const {
		return (SimbodyEngine*)(_dynamicsEngine);
	}

	Joint& operator=(const Joint &aJoint);
	void copyData(const Joint &aJoint);

	// GET & SET
	// Relating to the joint's body
	virtual void setBody(AbstractBody &aBody);
	virtual AbstractBody* getBody() const;
	virtual void setLocation(const SimTK::Vec3& aLocation);
	virtual void getLocation(SimTK::Vec3& rLocation) const;
	virtual void setOrientation(const SimTK::Vec3& aOrientation);
	virtual void getOrientation(SimTK::Vec3& rOrientation) const;
	// Relating to the parent body
	void setParentName(const std::string& aName);
	std::string getParentName() const;
	virtual void setParentBody(AbstractBody &aBody);
	virtual AbstractBody* getParentBody() const;
	virtual void setLocationInParent(const SimTK::Vec3& aLocation);
	virtual void getLocationInParent(SimTK::Vec3& rLocation) const;
	virtual void setOrientationInParent(const SimTK::Vec3& aOrientation);
	virtual void getOrientationInParent(SimTK::Vec3& rOrientation) const;

	virtual void getLocationInParent(double rLocation[]) const {
		_locationInParent.getAs(&rLocation[0]);
	};
	virtual void setLocationInChild(const SimTK::Vec3& aLocation) {
		_location = aLocation;
	};
	virtual void getLocationInChild(double rLocation[]) const {
		_location.getAs(&rLocation[0]);
	};
	virtual void getLocationInChild(SimTK::Vec3& rLocation) const {
		rLocation = _location;
	};
	// Coordinate Set
	virtual CoordinateSet* getCoordinateSet() const { return &_coordinateSet; }
	// Transforms
	virtual const Transform& getForwardTransform();
	virtual const Transform& getInverseTransform();

	//Model building
	/** Connect the body to its parent specified by this joint in the underlying Simbody 
	    model. If the parent is not connected (does not have a valid MobilzedBodyIndex) 
		then throw an exception. It is up to the assembly routine to make sure it is
		connecting in a valid sequence - not the joint. */
	virtual void connectBody();

	// Utility
	virtual bool isCoordinateUsed(AbstractCoordinate* aCoordinate) const;
	virtual bool hasXYZAxes() const { return true; }

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

protected:
	// Methods that allow access for Joint subclasses to data members of objects that
	// Joint befriends like Body, Coordinate and SimbodyEngine
	SimTK::MobilizedBodyIndex getMobilizedBodyIndex(Body *aBody) {return aBody->_index;} 
	void setMobilizedBodyIndex(Body *aBody, SimTK::MobilizedBodyIndex index) {aBody->_index = index;} 
	SimTK::MultibodySystem* getMultibodySystem(SimbodyEngine* engine);

	void associateCoordinatesAndSpeeds();

private:
	void setNull();
	void setupProperties();
	void updateSimbody();
	friend class SimbodyEngine;

//=============================================================================
};	// END of class Joint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Joint_h__


