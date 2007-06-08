// SimbodyBody.cpp
// Author: Frank C. Anderson
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
using namespace SimTK;
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
 * Update the underlying Simbody parameters, such as mass, to reflect any
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
//done_____________________________________________________________________________
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

	// TODO: Update Simbody mass
	// This is not currently supported except by rebuilding the entire Simbody model.
	cerr<<"SimbodyBody.setMass: updating Simbody not yet implemented.\n";

	// Update property
	_mass = aMass;

	return true;
}

//done_____________________________________________________________________________
/**
 * Get the mass center of the body.
 *
 * @param rVec XYZ coordinates of mass center are returned here.
 */
void SimbodyBody::getMassCenter(double rVec[3]) const
{
	Vec3 com = _engine->_matter.getBodyMassCenterStation(_engine->_s,_id);
	rVec[0] = com[0];
	rVec[1] = com[1];
	rVec[2] = com[2];
}
//_____________________________________________________________________________
/**
 * Set the mass center of the body.
 *
 * @param aVec XYZ coordinates of mass center.
 * @return Whether mass center was successfully changed.
 */
bool SimbodyBody::setMassCenter(const double aVec[3])
{
	// TODO: Update Simbody
	cerr<<"SimbodyBody.setMassCenter: updating Simbody not yet implemented.\n";

	// Update property
	_massCenter[0] = aVec[0];
	_massCenter[1] = aVec[1];
	_massCenter[2] = aVec[2];

	return true;
}

//done_____________________________________________________________________________
/**
 * Get the inertia matrix of the body.
 *
 * @param 3x3 inertia matrix.
 */
void SimbodyBody::getInertia(double rInertia[3][3]) const
{
	try {
		Inertia inertia = _engine->_matter.getBodyInertiaAboutBodyOrigin(_engine->_s,_id);
		Vec3 moments = inertia.getMoments();
		Vec3 products = inertia.getProducts();

		// TODO:  Verify that I have the order of the products correct.
		rInertia[0][0] = moments[0];
		rInertia[1][1] = moments[1];
		rInertia[2][2] = moments[2];
		rInertia[0][1] = rInertia[1][0] = products[0];
		rInertia[0][2] = rInertia[2][0] = products[1];
		rInertia[1][2] = rInertia[2][1] = products[2];
	} catch(const exception& e) {
		printf("EXCEPTION THROWN: %s\n", e.what());
	}
}
//done_____________________________________________________________________________
/**
 * Get the inertia matrix of the body.
 *
 * @param 3x3 inertia matrix.
 */
void SimbodyBody::getInertia(Array<double> &rInertia) const
{
	double inertia[3][3];
	getInertia(inertia);
	memcpy(&rInertia[0],&inertia[0][0],9*sizeof(double));
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
	// TODO: update Simbody inertial properties.
	cerr<<"SimbodyBody.setInertia: updating Simbody not yet implemented.\n";

	// Property
	_inertia = aInertia;

	return true;
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
	// Simbody
	// TODO: update Simbody
	cerr<<"SimbodyBody.setInertia: updating Simbody not yet implemented.\n";

	// Property
	memcpy(&_inertia[0],&aInertia[0][0],9*sizeof(double));

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
	cerr<<"SimbodyBody.scaleInertialProperties: not yet implemented.\n";
	return;

	// Scales assuming mass stays the same
	double mass, inertia[3][3];
	scaleInertiaTensor(mass, aScaleFactors, inertia);

	// Set new values
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
	cerr<<"SimbodyBody.scaleMass: not yet implemented.\n";
	return;

	double mass, inertia[3][3];

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
