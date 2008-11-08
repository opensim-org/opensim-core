// SimmBody.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
#include "SimmBody.h"
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Common/SimmMacros.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Mat33;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmBody::SimmBody() :
   _mass(_massProp.getValueDbl()),
   _massCenter(_massCenterProp.getValueDblVec3()),
   _inertia(_inertiaProp.getValueDblArray()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmBody::~SimmBody()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aBody SimmBody to be copied.
 */
SimmBody::SimmBody(const SimmBody &aBody) :
   AbstractBody(aBody),
   _mass(_massProp.getValueDbl()),
   _massCenter(_massCenterProp.getValueDblVec3()),
   _inertia(_inertiaProp.getValueDblArray()),
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
 * @return Pointer to a copy of this SimmBody.
 */
Object* SimmBody::copy() const
{
	SimmBody *body = new SimmBody(*this);
	return(body);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimmBody to another.
 *
 * @param aBody SimmBody to be copied.
 */
void SimmBody::copyData(const SimmBody &aBody)
{
	_mass = aBody._mass;
	_massCenter = aBody._massCenter;
	_inertia = aBody._inertia;
	_displayer = aBody._displayer; //? Do we need a dep copy here? when is this invoked?
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmBody to their null values.
 */
void SimmBody::setNull()
{
	setType("SimmBody");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmBody::setupProperties()
{
	_massProp.setName("mass");
	_massProp.setValue(0.0);
	_propertySet.append(&_massProp);

	const SimTK::Vec3 defaultMC(0.0, 0.0, 0.0);
	_massCenterProp.setName("mass_center");
	_massCenterProp.setValue(defaultMC);
	//_massCenterProp.setAllowableArraySize(3);
	_propertySet.append(&_massCenterProp);

	const double defaultInertia[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	_inertiaProp.setName("inertia");
	_inertiaProp.setValue(9, defaultInertia);
	_inertiaProp.setAllowableArraySize(9);
	_propertySet.append(&_inertiaProp);

	_displayerProp.setName("Displayer");
	_propertySet.append(&_displayerProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimmBody.
 */
void SimmBody::setup(AbstractDynamicsEngine* aEngine)
{
	// Base class
	AbstractBody::setup(aEngine);

	int i;
	for (i = 0; i < _displayer.getNumGeometryFiles(); i++)
		_displayer.addGeometry(new PolyhedralGeometry("bones/"+_displayer.getGeometryFileName(i)));

	_displayer.setOwner(this);
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
SimmBody& SimmBody::operator=(const SimmBody &aBody)
{
	// BASE CLASS
	AbstractBody::operator=(aBody);

	copyData(aBody);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the mass of the body.
 *
 * @param aMass mass of body.
 * @return Whether mass was successfully changed.
 */
bool SimmBody::setMass(double aMass)
{
	if (aMass >= 0.0)
	{
		_mass = aMass;
		return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Get the mass center of the body.
 *
 * @param rVec XYZ coordinates of mass center are returned here.
 */
void SimmBody::getMassCenter(SimTK::Vec3& rVec) const
{
	rVec = _massCenter;
}
 
//_____________________________________________________________________________
/**
 * Set the mass center of the body.
 *
 * @param aVec XYZ coordinates of mass center.
 * @return Whether mass center was successfully changed.
 */
bool SimmBody::setMassCenter(const SimTK::Vec3& aVec)
{
	_massCenter = aVec;

	return true;
}

//_____________________________________________________________________________
/**
 * Get the inertia matrix of the body.
 *
 * @param rInertia 3x3 inertia matrix.
 */
void SimmBody::getInertia(Mat33& rInertia) const
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			rInertia[i][j] = _inertia[i*3 + j];
}

//_____________________________________________________________________________
/**
 * Get the inertia matrix of the body.
 *
 * @param 1x9 inertia matrix.
 */
void SimmBody::getInertia(double rInertia[]) const
{
	memcpy(&rInertia[0], &_inertia[0], 9*sizeof(double));
}

//_____________________________________________________________________________
/**
 * Set the inertia matrix of the body.
 *
 * @param aInertia 9-element inertia matrix.
 * @return Whether inertia matrix was successfully changed.
 */
bool SimmBody::setInertia(const Mat33& aInertia)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		_inertia[i*3 + j] = aInertia[i][j];

	return true;
}

void SimmBody::setDisplayer(VisibleObject& aVisibleObject)
{
	_displayer = aVisibleObject;
}

//=============================================================================
// BONES
//=============================================================================
//_____________________________________________________________________________
/**
 * Add a bone to the body.
 *
 * @param aBone bone to be added.
void SimmBody::addBone(VisibleObject* aBone)
{
	VisibleObject* newBone = new VisibleObject(*aBone);

	// note: _boneSet takes over ownership of newBone
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
void SimmBody::scale(const Vec3& aScaleFactors, bool aScaleMass)
{
   // Base class, to scale wrap objects
   AbstractBody::scale(aScaleFactors, aScaleMass);

	SimTK::Vec3 displayerScaleFactors;
	getDisplayer()->getScaleFactors(displayerScaleFactors);

	for (int i = 0; i < 3; i++)
	{
		_massCenter[i] *= aScaleFactors[i];
		displayerScaleFactors[i] *= aScaleFactors[i];
	}
	// Update scale factors for displayer
	getDisplayer()->setScaleFactors(displayerScaleFactors);

	scaleInertialProperties(aScaleFactors, aScaleMass);

	//for (i = 0; i < _boneSet.getSize(); i++)
		//_boneSet.get(i)->scale(aScaleFactors);
}

//_____________________________________________________________________________
/**
 * Scale the body's inertia tensor and optionally the mass (represents a scaling
 * of the body's geometry).
 *
 * @param aScaleFactors XYZ scale factors.
 */
void SimmBody::scaleInertialProperties(const Vec3& aScaleFactors, bool aScaleMass)
{
	Mat33 inertia;
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			inertia[i][j] = _inertia[3*i+j];

	// Scales assuming mass stays the same
	scaleInertiaTensor(_mass, aScaleFactors, inertia);

	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			_inertia[3*i+j] = inertia[i][j];

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
void SimmBody::scaleMass(double aScaleFactor)
{
	_mass *= aScaleFactor;
	for (int i=0;i<9;i++) _inertia[i] *= aScaleFactor;
}

//=============================================================================
// ITERATORS FOR COMPONENTS
//=============================================================================
//_____________________________________________________________________________
/**
 * Make an iterator for the body's bone set.
 *
 * @return Pointer to the bone iterator.
BoneIterator* SimmBody::newBoneIterator() const
{
	return new BoneSetIterator(_boneSet);
}
 */

//=============================================================================
// I/O
//=============================================================================
void SimmBody::getScaleFactors(SimTK::Vec3& scales) const
{

	SimTK::Vec3 scaleFactors;
	_displayer.getScaleFactors(scaleFactors);

	scales = scaleFactors;

}
