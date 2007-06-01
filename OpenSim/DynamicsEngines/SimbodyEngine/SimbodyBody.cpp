// SimbodyBody.cpp
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
#include <iostream>
#include "SimbodyBody.h"
#include "SimbodyEngine.h"
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/SimmMacros.h>

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
SimbodyBody::SimbodyBody() :
	_mass(_massProp.getValueDbl()),
   _massCenter(_massCenterProp.getValueDblArray()),
	_inertia(_inertiaProp.getValueDblArray()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();
	setupProperties();
	updateSimbody();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodyBody::~SimbodyBody()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aBody SimbodyBody to be copied.
 */
SimbodyBody::SimbodyBody(const SimbodyBody &aBody) :
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
 * Copy constructor from an AbstractBody.
 *
 * @param aBody SimbodyBody to be copied.
 */
SimbodyBody::SimbodyBody(const AbstractBody &aBody) :
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
 * @return Pointer to a copy of this SimbodyBody.
 */
Object* SimbodyBody::copy() const
{
	SimbodyBody *body = new SimbodyBody(*this);
	return(body);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimbodyBody to another.
 *
 * @param aBody SimbodyBody to be copied.
 */
void SimbodyBody::copyData(const SimbodyBody &aBody)
{
	_mass = aBody._mass;
	_massCenter = aBody._massCenter;
	_inertia = aBody._inertia;
	_displayer = aBody._displayer;
	_id = aBody._id;
	_engine = aBody._engine;
	updateSimbody();
}

//_____________________________________________________________________________
/**
 * Copy data members from an AbstractBody to an SimbodyBody.
 *
 * @param aBody AbstractBody to be copied.
 */
void SimbodyBody::copyData(const AbstractBody &aBody)
{
	// Mass
	_mass = aBody.getMass();

	// Mass center
	aBody.getMassCenter(&_massCenter[0]);

	// Inertia tensor
	double inertia[3][3];
	aBody.getInertia(inertia);
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			_inertia[3*i+j] = inertia[i][j];

	// Displayer
	_displayer = *aBody.getDisplayer();

	// Update SDFast
	updateSimbody();
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimbodyBody to their null values.
 */
void SimbodyBody::setNull()
{
	setType("SimbodyBody");
	_engine = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimbodyBody::setupProperties()
{
	double mass = 1.0;
	_massProp.setName("mass");
	_massProp.setValue(mass);
	_propertySet.append(&_massProp);

	const double defaultMC[] = {0.0, 0.0, 0.0};
	_massCenterProp.setName("mass_center");
	_massCenterProp.setValue(3, defaultMC);
	_propertySet.append(&_massCenterProp);

	const double inertia[] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	_inertiaProp.setName("inertia");
	_inertiaProp.setValue(9,inertia);
	_propertySet.append(&_inertiaProp);

	_displayerProp.setName("Displayer");
	_propertySet.append(&_displayerProp);
}

//_____________________________________________________________________________
/**
 * Update the underlying SDFast parameters, such as mass, to reflect any
 * changes in the properties for this body.  This method should be called,
 * for example, after updateFromXMLNode() is called.
 */
void SimbodyBody::updateSimbody()
{
	setMass(_mass);
	setMassCenter(&_massCenter[0]);
	setInertia(_inertia);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimbodyBody.
 */
void SimbodyBody::setup(AbstractDynamicsEngine* aEngine)
{
	// Base class
	AbstractBody::setup(aEngine);

	_displayer.setOwner(this);

	_engine = dynamic_cast<SimbodyEngine*>(aEngine);

	updateSimbody();
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
SimbodyBody& SimbodyBody::operator=(const SimbodyBody &aBody)
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
 * Get the mass of the body.
 *
 * @return Mass of body from Simbody code.
 */
double SimbodyBody::getMass() const
{
	return _engine->_matter.getBodyMass(_engine->_s,_id);
}
//_____________________________________________________________________________
/**
 * Set the mass of the body.
 *
 * @param aMass mass of body.
 * @return Whether mass was successfully changed.
 */
bool SimbodyBody::setMass(double aMass)
{
	if(aMass<0.0) {
		cerr<<"SimbodyBody.setMass(): ERROR- zero or negative mass not allowed.\n";
		return false;
	}

	// Check to see if the mass is different
	double mass = getMass();
	if(rdMath::IsEqual(mass,aMass,rdMath::ZERO)) return true;

	// Update property
	_mass = aMass;

	// Update sdfast
	Q: how do I change the mass of a body?
		_engine->_matter.getArticulatedBodyInertia(_engine->_s,_id);

	return true;
}

//_____________________________________________________________________________
/**
 * Get the mass center of the body.
 *
 * @param rVec XYZ coordinates of mass center are returned here.
 */
void SimbodyBody::getMassCenter(double rVec[3]) const
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
bool SimbodyBody::setMassCenter(double aVec[3])
{
	_massCenter[0] = aVec[0];
	_massCenter[1] = aVec[1];
	_massCenter[2] = aVec[2];

	SimbodyEngine* engine = dynamic_cast<SimbodyEngine*>(_dynamicsEngine);
	if(engine)
		return engine->adjustJointVectorsForNewMassCenter(this);
	else
		return false;
}

//_____________________________________________________________________________
/**
 * Get the inertia matrix of the body.
 *
 * @param 3x3 inertia matrix.
 */
void SimbodyBody::getInertia(double rInertia[3][3]) const
{
	if(_index<0) {
		for(int i=0; i<3; i++) {
			for(int j=0; j<3; j++) {
				rInertia[i][j] = 0;
			}
		}
	} else {
		_engine->_sdgetiner(_index,rInertia);
	}
}
//_____________________________________________________________________________
/**
 * Get the inertia matrix of the body.
 *
 * @param 3x3 inertia matrix.
 */
void SimbodyBody::getInertia(Array<double> &rInertia) const
{
	double inertia[3][3];
	getInertia(inertia);
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			rInertia[i*3+j] = inertia[i][j];
		}
	}
}
//_____________________________________________________________________________
/**
 * Set the inertia matrix of the body.
 *
 * @param aInertia 9-element inertia matrix.
 * @return Whether inertia matrix was successfully changed.
 */
bool SimbodyBody::setInertia(const Array<double>& aInertia)
{
	if(aInertia.getSize()<9) {
		cerr<<"SimbodyBody.setInertia: ERROR- inertia requires 9 elements.\n";
		return false;
	}

	double inertia[3][3];
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			inertia[i][j] = aInertia[3*i+j];

	return setInertia(inertia);
}
//_____________________________________________________________________________
/**
 * Set the inertia matrix of the body.
 *
 * @param aInertia 9-element inertia matrix.
 * @return Whether inertia matrix was successfully changed.
 */
bool SimbodyBody::setInertia(const double aInertia[3][3])
{
	if(_index<0) return false;

	// Check to see if the inertia is different from what SDFast already has
	bool same = true;
	double inertia[3][3];
	getInertia(inertia);
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			if(!rdMath::IsEqual(inertia[i][j],aInertia[i][j],rdMath::ZERO)) same = false;
	if(same==true) return true;

	// Update property
	//cout<<"SimbodyBody.setInertia: body="<<getName()<<"\n\torig=";
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			inertia[i][j] = aInertia[i][j]; // to remove the const'ness (I think)
			_inertia[i*3+j] = aInertia[i][j]; // set local member
		}
	}

	_engine->_sdiner(_index,inertia);
	_engine->_sdinit();

	return true;
}

//=============================================================================
// BONES
//=============================================================================
//_____________________________________________________________________________
/**
 * Add a bone to the body.
 *
 * @param aBone bone to be added.
void SimbodyBody::addBone(VisibleObject* aBone)
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
void SimbodyBody::scale(const Array<double>& aScaleFactors, bool aScaleMass)
{
	int i;

	double oldScaleFactors[3];
	getDisplayer()->getScaleFactors(oldScaleFactors);

	for(i=0; i<3; i++) {
		_massCenter[i] *= aScaleFactors[i];
		oldScaleFactors[i] *= aScaleFactors[i];
	}
	// Update scale factors for displayer
	getDisplayer()->setScaleFactors(aScaleFactors.get());

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
void SimbodyBody::scaleInertialProperties(const Array<double>& aScaleFactors, bool aScaleMass)
{
	double mass, inertia[3][3];

	_engine->_sdgetmass(_index, &mass);
	_engine->_sdgetiner(_index, inertia);

	// Scales assuming mass stays the same
	scaleInertiaTensor(mass, aScaleFactors, inertia);

	// Update properties and SDFast
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
void SimbodyBody::scaleMass(double aScaleFactor)
{
	double mass, inertia[3][3];

	_engine->_sdgetmass(_index, &mass);
	_engine->_sdgetiner(_index, inertia);

	mass *= aScaleFactor;
	for (int i=0;i<3;i++)
	  for(int j=0;j<3;j++)
		inertia[i][j] *= aScaleFactor;

	// Update properties and SDFast
	setMass(mass);
	setInertia(inertia);
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Transform a point from the body's frame to the body's SD/FAST frame.
 * This entails subtracting the mass center coordinates from the point
 * because the SD/FAST frame is lined up with the body frame, but has
 * its origin at the center of mass.
 *
 * @param aPos The point in the body frame to transform
 * @param rPos The point transformed into the SD/FAST frame
 */
void SimbodyBody::transformToSimbodyFrame(const double aPos[3],double rPos[3]) const
{
	for(int i=0; i<3; i++)
		rPos[i] = aPos[i] - _massCenter[i];
}

//_____________________________________________________________________________
/**
 * Transform a point from the body's frame to the body's SD/FAST frame.
 * This entails subtracting the mass center coordinates from the point
 * because the SD/FAST frame is lined up with the body frame, but has
 * its origin at the center of mass.
 *
 * @param aPos The point in the body frame to transform
 * @param rPos The point transformed into the SD/FAST frame
 */
void SimbodyBody::transformToSimbodyFrame(const Array<double>& aPos, double rPos[3]) const
{
	for(int i=0; i<3; i++)
		rPos[i] = aPos[i] - _massCenter[i];
}

//_____________________________________________________________________________
/**
 * Transform a point from the body's SD/FAST frame to the body's frame.
 * This entails adding the mass center coordinates to the point
 * because the SD/FAST frame is lined up with the body frame, but has
 * its origin at the center of mass.
 *
 * @param aPos The point in the body's SD/FAST frame to transform
 * @param rPos The point transformed into the body's frame
 */
void SimbodyBody::transformFromSimbodyFrame(const double aPos[3], double rPos[3]) const
{
	for(int i=0; i<3; i++)
		rPos[i] = aPos[i] + _massCenter[i];
}

//_____________________________________________________________________________
/**
 * Transform a point from the body's SD/FAST frame to the body's frame.
 * This entails adding the mass center coordinates to the point
 * because the SD/FAST frame is lined up with the body frame, but has
 * its origin at the center of mass.
 *
 * @param aPos The point in the body's SD/FAST frame to transform
 * @param rPos The point transformed into the body's frame
 */
void SimbodyBody::transformFromSimbodyFrame(const Array<double>& aPos, double rPos[3]) const
{
	for(int i=0; i<3; i++)
		rPos[i] = aPos[i] + _massCenter[i];
}

void SimbodyBody::peteTest() const
{
	cout << "Body: " << getName() << endl;
	cout << "   massCenter: " << _massCenter << endl;
}
