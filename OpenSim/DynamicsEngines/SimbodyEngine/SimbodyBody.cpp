// SimbodyBody.cpp
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
   _massCenter(_massCenterProp.getValueDblVec3()),
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
 * Copy constructor from an AbstractBody.
 *
 * @param aBody SimbodyBody to be copied.
 */
SimbodyBody::SimbodyBody(const AbstractBody &aBody) :
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
}

//_____________________________________________________________________________
/**
 * Update the underlying Simbody parameters, such as mass, to reflect any
 * changes in the properties for this body.  This method should be called,
 * for example, after updateFromXMLNode() is called.
 */
void SimbodyBody::updateSimbody()
{
	//setMass(_mass);
	//setMassCenter(&_massCenter[0]);
	//setInertia(_inertia);
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
bool SimbodyBody::setMass(double aMass)
{
	if(aMass<0.0) {
		cerr<<"SimbodyBody.setMass(): ERROR- zero or negative mass not allowed.\n";
		return false;
	}
	_mass = aMass;
	_engine->constructMultibodySystem();

	return true;
}

//done_____________________________________________________________________________
/**
 * Get the mass center of the body.
 *
 * @param rVec XYZ coordinates of mass center are returned here.
 */
void SimbodyBody::getMassCenter(SimTK::Vec3& rVec) const
{
	memcpy(&rVec[0],&_massCenter[0],3*sizeof(double));
}
//_____________________________________________________________________________
/**
 * Set the mass center of the body.
 *
 * @param aVec XYZ coordinates of mass center.
 * @return Whether mass center was successfully changed.
 */
bool SimbodyBody::setMassCenter(const SimTK::Vec3& aVec)
{
	_massCenter=aVec;
	_engine->constructMultibodySystem();

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
	/*
	try {
		Inertia inertia = _engine->_matter->getBodyInertiaAboutBodyOrigin(_engine->_s,_id);
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
	*/
	memcpy(&rInertia[0][0],&_inertia[0],9*sizeof(double));
}
//done_____________________________________________________________________________
/**
 * Get the inertia matrix of the body.
 *
 * @param 3x3 inertia matrix.
 */
void SimbodyBody::getInertia(Mat33 &rInertia) const
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
bool SimbodyBody::setInertia(const Mat33& aInertia)
{
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			_inertia[3*i+j] = aInertia[i][j];
	_engine->constructMultibodySystem();

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
	memcpy(&_inertia[0],&aInertia[0][0],9*sizeof(double));
	_engine->constructMultibodySystem();

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
void SimbodyBody::scale(const SimTK::Vec3& aScaleFactors, bool aScaleMass)
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
void SimbodyBody::scaleInertialProperties(const SimTK::Vec3& aScaleFactors, bool aScaleMass)
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
void SimbodyBody::scaleMass(double aScaleFactor)
{
	_mass *= aScaleFactor;
	for (int i=0;i<9;i++) _inertia[i] *= aScaleFactor;
}


//=============================================================================
// UTILITY
//=============================================================================


//=============================================================================
// I/O
//=============================================================================
void SimbodyBody::getScaleFactors(Vec3& scales) const
{

	SimTK::Vec3 scaleFactors;
	_displayer.getScaleFactors(scaleFactors);

	scales = scaleFactors;

}
