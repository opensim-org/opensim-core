// Body.cpp
// Author: Frank C. Anderson
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include "Body.h"
#include "SimbodyEngine.h"
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/SimmMacros.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
//using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Body::~Body()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
Body::Body() :
	_mass(_massProp.getValueDbl()),
	_massCenter(_massCenterProp.getValueDblVec3()),
	_inertiaXX(_inertiaXXProp.getValueDbl()),
	_inertiaYY(_inertiaYYProp.getValueDbl()),
	_inertiaZZ(_inertiaZZProp.getValueDbl()),
	_inertiaXY(_inertiaXYProp.getValueDbl()),
	_inertiaXZ(_inertiaXZProp.getValueDbl()),
	_inertiaYZ(_inertiaYZProp.getValueDbl()),
	_joint(_jointProp.getValueObjPtrRef()),
	_displayerProp(PropertyObj("", VisibleObject())),
	_displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();
	setupProperties();
	//updateSimbody();
}

	//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aBody Body to be copied.
 */
Body::Body(const Body &aBody) :
   AbstractBody(aBody),
	_mass(_massProp.getValueDbl()),
	_massCenter(_massCenterProp.getValueDblVec3()),
	_inertiaXX(_inertiaXXProp.getValueDbl()),
	_inertiaYY(_inertiaYYProp.getValueDbl()),
	_inertiaZZ(_inertiaZZProp.getValueDbl()),
	_inertiaXY(_inertiaXYProp.getValueDbl()),
	_inertiaXZ(_inertiaXZProp.getValueDbl()),
	_inertiaYZ(_inertiaYZProp.getValueDbl()),
	_joint(_jointProp.getValueObjPtrRef()),
	_displayerProp(PropertyObj("", VisibleObject())),
	_displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();
	setupProperties();
	copyData(aBody);
}

//_____________________________________________________________________________
/**
 * Copy constructor from an AbstractBody.
 *
 * @param aBody Body to be copied.
 */
Body::Body(const AbstractBody &aBody) :
   AbstractBody(aBody),
	_mass(_massProp.getValueDbl()),
	_massCenter(_massCenterProp.getValueDblVec3()),
	_inertiaXX(_inertiaXXProp.getValueDbl()),
	_inertiaYY(_inertiaYYProp.getValueDbl()),
	_inertiaZZ(_inertiaZZProp.getValueDbl()),
	_inertiaXY(_inertiaXYProp.getValueDbl()),
	_inertiaXZ(_inertiaXZProp.getValueDbl()),
	_inertiaYZ(_inertiaYZProp.getValueDbl()),
	_joint(_jointProp.getValueObjPtrRef()),
	_displayerProp(PropertyObj("", VisibleObject())),
	_displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();
	setupProperties();
	copyData(aBody);
}

//_____________________________________________________________________________
/**
 * Copy this body and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this Body.
 */
Object* Body::copy() const
{
	Body *body = new Body(*this);
	return(body);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one Body to another.
 *
 * @param aBody Body to be copied.
 */
void Body::copyData(const Body &aBody)
{
	_mass = aBody._mass;
	_massCenter = aBody._massCenter;
	_inertiaXX = aBody._inertiaXX;
	_inertiaYY = aBody._inertiaYY;
	_inertiaZZ = aBody._inertiaZZ;
	_inertiaXY = aBody._inertiaXY;
	_inertiaXZ = aBody._inertiaXZ;
	_inertiaYZ = aBody._inertiaYZ;
	_displayer = aBody._displayer;
	_index = aBody._index;
	_joint = dynamic_cast<Joint*>(Object::SafeCopy(aBody._joint));
	_dynamicsEngine = aBody._dynamicsEngine;
	//updateSimbody();
}

//_____________________________________________________________________________
/**
 * Copy data members from an AbstractBody to an Body.
 *
 * @param aBody AbstractBody to be copied.
 */
void Body::copyData(const AbstractBody &aBody)
{
	// Mass
	_mass = aBody.getMass();

	// Mass center
	aBody.getMassCenter(_massCenter);

	// Inertia tensor
	SimTK::Mat33 inertia;
	aBody.getInertia(inertia);
	setInertia(inertia);

	// Joint
	//_joint = aBody._joint;  Problem? Abstract bodies do not have a joint?

	// Displayer
	_displayer = *aBody.getDisplayer();

	// Update SDFast
	//updateSimbody();
}

//_____________________________________________________________________________
/**
 * Set the data members of this Body to their null values.
 */
void Body::setNull()
{
	setType("Body");
	_dynamicsEngine = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Body::setupProperties()
{
	double mass = 1.0;
	_massProp.setName("mass");
	_massProp.setValue(mass);
	_propertySet.append(&_massProp);

	const SimTK::Vec3 defaultMC(0.0, 0.0, 0.0);
	_massCenterProp.setName("mass_center");
	_massCenterProp.setValue(defaultMC);
	//_massCenterProp.setAllowableArraySize(3);
	_propertySet.append(&_massCenterProp);

	// Ixx
	_inertiaXXProp.setName("inertia_xx");
	_inertiaXXProp.setValue(1.0);
	_propertySet.append(&_inertiaXXProp);

	// Iyy
	_inertiaYYProp.setName("inertia_yy");
	_inertiaYYProp.setValue(1.0);
	_propertySet.append(&_inertiaYYProp);

	// Izz
	_inertiaZZProp.setName("inertia_zz");
	_inertiaZZProp.setValue(1.0);
	_propertySet.append(&_inertiaZZProp);

	// Ixy
	_inertiaXYProp.setName("inertia_xy");
	_inertiaXYProp.setValue(0.0);
	_propertySet.append(&_inertiaXYProp);

	// Ixz
	_inertiaXZProp.setName("inertia_xz");
	_inertiaXZProp.setValue(0.0);
	_propertySet.append(&_inertiaXZProp);

	// Iyz
	_inertiaYZProp.setName("inertia_yz");
	_inertiaYZProp.setValue(0.0);
	_propertySet.append(&_inertiaYZProp);

	// Joint
	_jointProp.setComment("Joint that connects this body with the parent body.");
	_jointProp.setName("Joint");
	_propertySet.append(&_jointProp);

	_displayerProp.setName("Displayer");
	_propertySet.append(&_displayerProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this Body.
 */
void Body::setup(AbstractDynamicsEngine* aEngine)
{
	AbstractBody::setup(aEngine);
	_displayer.setOwner(this);
	//_dynamicsEngine = dynamic_cast<SimbodyEngine*>(aEngine);
	//updateSimbody();
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Body& Body::operator=(const Body &aBody)
{
	// BASE CLASS
	AbstractBody::operator=(aBody);

	copyData(aBody);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//done_____________________________________________________________________________
/**
 * Get the mass of the body.
 *
 * @return Mass of body from Simbody code.
 */
double Body::getMass() const
{
	return _mass;
	//return _engine->_matter->getBodyMass(_engine->_s,_id);
}
//_____________________________________________________________________________
/**
 * Set the mass of the body.
 *
 * @param aMass mass of body.
 * @return Whether mass was successfully changed.
 */
bool Body::setMass(double aMass)
{
	if(aMass<0.0) {
		cerr<<"Body.setMass(): ERROR- zero or negative mass not allowed.\n";
		return false;
	}
	_mass = aMass;
	getEngine()->updateBodyInertia(this);
	return true;
}

//done_____________________________________________________________________________
/**
 * Get the mass center of the body.
 *
 * @param rVec XYZ coordinates of mass center are returned here.
 */
void Body::getMassCenter(SimTK::Vec3& rVec) const
{
	rVec=_massCenter;
}
//_____________________________________________________________________________
/**
 * Set the mass center of the body.
 *
 * @param aVec XYZ coordinates of mass center.
 * @return Whether mass center was successfully changed.
 */
bool Body::setMassCenter(const SimTK::Vec3& aVec)
{
	_massCenter=aVec;
	getEngine()->updateBodyInertia(this);
	return true;
}

//_____________________________________________________________________________
/**
 * Get the inertia matrix of the body.
 *
 * @param 3x3 inertia matrix.
 */
void Body::getInertia(SimTK::Mat33 &rInertia) const
{
	rInertia[0][0] = _inertiaXX;
	rInertia[0][1] = _inertiaXY;
	rInertia[0][2] = _inertiaXZ;
	rInertia[1][0] = _inertiaXY;
	rInertia[1][1] = _inertiaYY;
	rInertia[1][2] = _inertiaYZ;
	rInertia[2][0] = _inertiaXZ;
	rInertia[2][1] = _inertiaYZ;
	rInertia[2][2] = _inertiaZZ;
}
//_____________________________________________________________________________
/**
 * Set the inertia matrix of the body.
 *
 * @param aInertia 9-element inertia matrix.
 * @return Whether inertia matrix was successfully changed.
 */
bool Body::setInertia(const SimTK::Mat33& aInertia)
{
	_inertiaXX = aInertia[0][0];
	_inertiaXY = aInertia[0][1];
	_inertiaXZ = aInertia[0][2];
	_inertiaYY = aInertia[1][1];
	_inertiaYZ = aInertia[1][2];
	_inertiaZZ = aInertia[2][2];
	getEngine()->updateBodyInertia(this);
	return true;
}

//_____________________________________________________________________________
/**
 * Set the joint for this body.
 *
 * @param aJoint Joint connecting this body to the parent body.
 */
void Body::
setJoint(const Joint *aJoint)
{
	_joint = (Joint *) aJoint->copy();
}
	



//=============================================================================
// BONES
//=============================================================================
//_____________________________________________________________________________
/**
 * Add a bone to the body.
 *
 * @param aBone bone to be added.
void Body::addBone(VisibleObject* aBone)
{
	VisibleObject* newBone = new VisibleObject(*aBone);

	// note: _boneSet takes over ownership of newMarker
	_boneSet.append(newBone);
}
 */

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale the body.
 *
 * @param aScaleFactors XYZ scale factors.
 * @param aScaleMass whether or not to scale mass properties
 */
void Body::scale(const SimTK::Vec3& aScaleFactors, bool aScaleMass)
{
   // Base class, to scale wrap objects
   AbstractBody::scale(aScaleFactors, aScaleMass);

	SimTK::Vec3 oldScaleFactors;
	getDisplayer()->getScaleFactors(oldScaleFactors);

	for(int i=0; i<3; i++) {
		_massCenter[i] *= aScaleFactors[i];
		oldScaleFactors[i] *= aScaleFactors[i];
	}
	// Update scale factors for displayer
	getDisplayer()->setScaleFactors(oldScaleFactors);

	if (_index!=0)	// The following throws an exception if applied to ground.
		scaleInertialProperties(aScaleFactors, aScaleMass);
}

//_____________________________________________________________________________
/**
 * Scale the body's inertia tensor and optionally the mass (represents a scaling
 * of the body's geometry).
 *
 *
 * @param aScaleFactors XYZ scale factors.
 */
void Body::scaleInertialProperties(const SimTK::Vec3& aScaleFactors, bool aScaleMass)
{
	// Inertia
	SimTK::Mat33 inertia;
	getInertia(inertia);
	scaleInertiaTensor(_mass, aScaleFactors, inertia);
	setInertia(inertia);

	// Scales mass
	if(aScaleMass)
		scaleMass(DABS(aScaleFactors[0] * aScaleFactors[1] * aScaleFactors[2]));
}

//_____________________________________________________________________________
/**
 * Scale the body's mass and inertia tensor (represents a scaling of the
 * body's density).
 *
 * @param aScaleFactors XYZ scale factors.
 */
void Body::scaleMass(double aScaleFactor)
{
	if (_index==0)	// The following throws an exception if applied to ground.
		return;

	_mass *= aScaleFactor;
	_inertiaXX *= aScaleFactor;
	_inertiaYY *= aScaleFactor;
	_inertiaZZ *= aScaleFactor;
	_inertiaXY *= aScaleFactor;
	_inertiaXZ *= aScaleFactor;
	_inertiaYZ *= aScaleFactor;
	getEngine()->updateBodyInertia(this);
}


//=============================================================================
// UTILITY
//=============================================================================
SimTK::MassProperties Body::getMassProperties()
{
	SimTK::Inertia inertiaAboutCOM = SimTK::Inertia(_inertiaXX, _inertiaYY, _inertiaZZ,
													_inertiaXY, _inertiaXZ, _inertiaYZ);
	SimTK::Inertia inertiaAboutOrigin = inertiaAboutCOM.shiftFromMassCenter(-_massCenter, _mass);
	return SimTK::MassProperties(_mass, _massCenter, inertiaAboutOrigin);
}

//=============================================================================
// I/O
//=============================================================================
void Body::getScaleFactors(SimTK::Vec3& scales) const
{

	SimTK::Vec3 scaleFactors;
	_displayer.getScaleFactors(scaleFactors);

	scales = scaleFactors;

}
