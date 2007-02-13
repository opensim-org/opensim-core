// SimmBody.cpp
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "SimmBody.h"
#include "AbstractDynamicsEngine.h"
#include "SimmMacros.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmBody::SimmBody() :
   _mass(_massProp.getValueDbl()),
   _massCenter(_massCenterProp.getValueDblArray()),
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
   _massCenter(_massCenterProp.getValueDblArray()),
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

	const double defaultMC[] = {0.0, 0.0, 0.0};
	_massCenterProp.setName("mass_center");
	_massCenterProp.setValue(3, defaultMC);
	_propertySet.append(&_massCenterProp);

	const double defaultInertia[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	_inertiaProp.setName("inertia");
	_inertiaProp.setValue(9, defaultInertia);
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
void SimmBody::getMassCenter(double rVec[3]) const
{
	rVec[0] = _massCenter[0];
	rVec[1] = _massCenter[1];
	rVec[2] = _massCenter[2];
}
 
//_____________________________________________________________________________
/**
 * Set the mass center of the body.
 *
 * @param aVec XYZ coordinates of mass center.
 * @return Whether mass center was successfully changed.
 */
bool SimmBody::setMassCenter(double aVec[3])
{
	_massCenter[0] = aVec[0];
	_massCenter[1] = aVec[1];
	_massCenter[2] = aVec[2];

	return true;
}

//_____________________________________________________________________________
/**
 * Get the inertia matrix of the body.
 *
 * @param rInertia 3x3 inertia matrix.
 */
void SimmBody::getInertia(double rInertia[3][3]) const
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			rInertia[i][j] = _inertia[i*3 + j];
}

//_____________________________________________________________________________
/**
 * Set the inertia matrix of the body.
 *
 * @param aInertia 9-element inertia matrix.
 * @return Whether inertia matrix was successfully changed.
 */
bool SimmBody::setInertia(const Array<double>& aInertia)
{
	if (aInertia.getSize() >= 9)
	{
		for (int i = 0; i < 9; i++)
			_inertia[i] = aInertia[i];

		return true;
	}

	return false;
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
void SimmBody::scale(const Array<double>& aScaleFactors, bool aScaleMass)
{
	int i;

	double oldScaleFactors[3];
	getDisplayer()->getScaleFactors(oldScaleFactors);

	for (i = 0; i < 3; i++)
	{
		_massCenter[i] *= aScaleFactors[i];
		oldScaleFactors[i] *= aScaleFactors[i];
	}
	// Update scale factors for displayer
	getDisplayer()->setScaleFactors(aScaleFactors.get());

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
void SimmBody::scaleInertialProperties(const Array<double>& aScaleFactors, bool aScaleMass)
{
	double inertia[3][3];
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
void SimmBody::getScaleFactors(Array<double>& scales) const
{

	double scaleFactors[3];
	_displayer.getScaleFactors(scaleFactors);

	for (int i=0; i<3; i++)
		scales[i] = scaleFactors[i];

}

void SimmBody::peteTest() const
{
	cout << "Body: " << getName() << endl;
	cout << "   mass: " << _mass << endl;
	cout << "   massCenter: " << _massCenter << endl;
	cout << "   inertia: " << _inertia << endl;

	if (_wrapObjectSet.getSize() > 0) {
		int i;
		for (i = 0; i < _wrapObjectSet.getSize(); i++)
			_wrapObjectSet.get(i)->peteTest();
	} else {
		cout << "   no wrap objects" << endl;
	}
}
