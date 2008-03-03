// SdfastBody.cpp
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
#include <iostream>
#include "SdfastBody.h"
#include "SdfastEngine.h"
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/VisibleObject.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Mat33;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SdfastBody::SdfastBody() :
	_mass(_massProp.getValueDbl()),
   _massCenter(_massCenterProp.getValueDblVec3()),
	_inertia(_inertiaProp.getValueDblArray()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj()),
	_index(_indexProp.getValueInt()),
	_SdfastEngine(NULL)
{
	setNull();
	setupProperties();
	updateSdfast();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SdfastBody::~SdfastBody()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aBody SdfastBody to be copied.
 */
SdfastBody::SdfastBody(const SdfastBody &aBody) :
   AbstractBody(aBody),
	_mass(_massProp.getValueDbl()),
   _massCenter(_massCenterProp.getValueDblVec3()),
	_inertia(_inertiaProp.getValueDblArray()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj()),
	_index(_indexProp.getValueInt()),
	_SdfastEngine(NULL)
{
	setNull();
	setupProperties();
	copyData(aBody);
}

//_____________________________________________________________________________
/**
 * Copy constructor from an AbstractBody.
 *
 * @param aBody SdfastBody to be copied.
 */
SdfastBody::SdfastBody(const AbstractBody &aBody) :
   AbstractBody(aBody),
	_mass(_massProp.getValueDbl()),
   _massCenter(_massCenterProp.getValueDblVec3()),
	_inertia(_inertiaProp.getValueDblArray()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj()),
	_index(_indexProp.getValueInt()),
	_SdfastEngine(NULL)
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
 * @return Pointer to a copy of this SdfastBody.
 */
Object* SdfastBody::copy() const
{
	SdfastBody *body = new SdfastBody(*this);
	return(body);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SdfastBody to another.
 *
 * @param aBody SdfastBody to be copied.
 */
void SdfastBody::copyData(const SdfastBody &aBody)
{
	_mass = aBody._mass;
	_massCenter = aBody._massCenter;
	_inertia = aBody._inertia;
	_displayer = aBody._displayer;
	_index = aBody._index;
	_SdfastEngine = aBody._SdfastEngine; // TODO: should we be copying pointers?
	updateSdfast(); // TODO: should we really call this here?  or maybe wait until setup is called?
}

//_____________________________________________________________________________
/**
 * Copy data members from an AbstractBody to an SdfastBody.
 *
 * @param aBody AbstractBody to be copied.
 */
void SdfastBody::copyData(const AbstractBody &aBody)
{
	// Mass
	_mass = aBody.getMass();

	// Mass center
	aBody.getMassCenter(_massCenter);

	// Inertia tensor
	Mat33 inertia;
	aBody.getInertia(inertia);
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			_inertia[3*i+j] = inertia[i][j];

	// Displayer
	_displayer = *aBody.getDisplayer();

	// Update SDFast
	updateSdfast();
}

//_____________________________________________________________________________
/**
 * Set the data members of this SdfastBody to their null values.
 */
void SdfastBody::setNull()
{
	setType("SdfastBody");
	_SdfastEngine = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SdfastBody::setupProperties()
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

	const double inertia[] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	_inertiaProp.setName("inertia");
	_inertiaProp.setValue(9,inertia);
	_inertiaProp.setAllowableArraySize(9);
	_propertySet.append(&_inertiaProp);

	_displayerProp.setName("Displayer");
	_propertySet.append(&_displayerProp);

	_indexProp.setName("index");
	_indexProp.setValue(-2);
	_propertySet.append(&_indexProp);
}

//_____________________________________________________________________________
/**
 * Update the underlying SDFast parameters, such as mass, to reflect any
 * changes in the properties for this body.  This method should be called,
 * for example, after updateFromXMLNode() is called.
 */
void SdfastBody::updateSdfast()
{
	setMass(_mass);
	setMassCenter(_massCenter);
	Mat33 inertiaMat;
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			inertiaMat[i][j]=_inertia[i*3+j];
	setInertia(inertiaMat);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SdfastBody.
 */
void SdfastBody::setup(AbstractDynamicsEngine* aEngine)
{
	// Base class
	AbstractBody::setup(aEngine);

	_displayer.setOwner(this);

	_SdfastEngine = dynamic_cast<SdfastEngine*>(aEngine);

	updateSdfast();
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
SdfastBody& SdfastBody::operator=(const SdfastBody &aBody)
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
 * @return Mass of body from SD/FAST code.
 */
double SdfastBody::getMass() const
{
	if(_index<0) return 0.0;
	double mass = 0.0;
	_SdfastEngine->_sdgetmass(_index,&mass);
	return mass;
}
//_____________________________________________________________________________
/**
 * Set the mass of the body.
 *
 * @param aMass mass of body.
 * @return Whether mass was successfully changed.
 */
bool SdfastBody::setMass(double aMass)
{
	if(_index<0) return false;

	if(aMass<0.0) {
		cerr<<"SdfastBody.setMass(): ERROR- zero or negative mass not allowed.\n";
		return false;
	}

	// Check to see if the mass is different from what SDFast already has
	double mass = getMass();
	if(rdMath::IsEqual(mass,aMass,rdMath::ZERO)) return true;

	// Update property
	_mass = aMass;

	// Update sdfast
	_SdfastEngine->_sdmass(_index,aMass);
	_SdfastEngine->_sdinit();
	//cout<<"SdfastBody.setMass: body="<<getName()<<"  orig="<<mass<<"  new="<<_mass<<endl;

	return true;
}

//_____________________________________________________________________________
/**
 * Get the mass center of the body.
 *
 * @param rVec XYZ coordinates of mass center are returned here.
 */
void SdfastBody::getMassCenter(SimTK::Vec3& rVec) const
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
bool SdfastBody::setMassCenter(const SimTK::Vec3& aVec)
{
	_massCenter = aVec;

	SdfastEngine* engine = dynamic_cast<SdfastEngine*>(_dynamicsEngine);
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
void SdfastBody::getInertia(double rInertia[3][3]) const
{
	if(_index<0) {
		for(int i=0; i<3; i++) {
			for(int j=0; j<3; j++) {
				rInertia[i][j] = 0;
			}
		}
	} else {
		_SdfastEngine->_sdgetiner(_index,rInertia);
	}
}
//_____________________________________________________________________________
/**
 * Get the inertia matrix of the body.
 *
 * @param 3x3 inertia matrix.
 */
void SdfastBody::getInertia(Mat33 &rInertia) const
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
bool SdfastBody::setInertia(const Mat33& aInertia)
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		_inertia[i*3 + j] = aInertia[i][j];

	return true;
}
//_____________________________________________________________________________
/**
 * Set the inertia matrix of the body.
 *
 * @param aInertia 9-element inertia matrix.
 * @return Whether inertia matrix was successfully changed.
 */
bool SdfastBody::setInertia(const double aInertia[3][3])
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
	//cout<<"SdfastBody.setInertia: body="<<getName()<<"\n\torig=";
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			inertia[i][j] = aInertia[i][j]; // to remove the const'ness (I think)
			_inertia[i*3+j] = aInertia[i][j]; // set local member
		}
	}

	_SdfastEngine->_sdiner(_index,inertia);
	_SdfastEngine->_sdinit();

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
void SdfastBody::addBone(VisibleObject* aBone)
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
void SdfastBody::scale(const SimTK::Vec3& aScaleFactors, bool aScaleMass)
{
	int i;

	SimTK::Vec3 oldScaleFactors;
	getDisplayer()->getScaleFactors(oldScaleFactors);

	for(i=0; i<3; i++) {
		_massCenter[i] *= aScaleFactors[i];
		oldScaleFactors[i] *= aScaleFactors[i];
	}
	// Update scale factors for displayer
	getDisplayer()->setScaleFactors(aScaleFactors);

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
void SdfastBody::scaleInertialProperties(const SimTK::Vec3& aScaleFactors, bool aScaleMass)
{
	double mass;
	double inertia[3][3];

	_SdfastEngine->_sdgetmass(_index, &mass);
	_SdfastEngine->_sdgetiner(_index, inertia);

	for(int i=0; i<3; i++) for (int j=0; j<3; j++) 
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
void SdfastBody::scaleMass(double aScaleFactor)
{
	double mass, inertia[3][3];

	_SdfastEngine->_sdgetmass(_index, &mass);
	_SdfastEngine->_sdgetiner(_index, inertia);

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
void SdfastBody::transformToSdfastFrame(const double aPos[3],double rPos[3]) const
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
void SdfastBody::transformToSdfastFrame(const SimTK::Vec3& aPos, SimTK::Vec3& rPos) const
{
	rPos = aPos - _massCenter;
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
void SdfastBody::transformFromSdfastFrame(const double aPos[3], double rPos[3]) const
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
void SdfastBody::transformFromSdfastFrame(const SimTK::Vec3& aPos, SimTK::Vec3& rPos) const
{
	rPos = aPos + _massCenter;
}
