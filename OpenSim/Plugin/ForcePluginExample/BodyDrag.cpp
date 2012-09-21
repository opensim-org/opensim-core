/* -------------------------------------------------------------------------- *
 *                           OpenSim:  BodyDrag.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Tim Dorn                                                        *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "BodyDrag.h"

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
 * Destructor.
 */
BodyDrag::~BodyDrag()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
BodyDrag::BodyDrag() : Force(),
	_bodyName(_bodyNameProp.getValueStr()),
	_coefficient(_coefficientProp.getValueDbl()),
	_exponent(_exponentProp.getValueDbl())
{
	setNull();
}

/* Convenience constructor */
BodyDrag::BodyDrag( std::string bodyName, double coefficient, double exponent):	Force(),
	_bodyName(_bodyNameProp.getValueStr()),
	_coefficient(_coefficientProp.getValueDbl()),
	_exponent(_exponentProp.getValueDbl())
{
	setNull();
	_bodyName = bodyName;
	_coefficient = coefficient;
	_exponent = exponent;
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce BodyDrag to be copied.
 */
BodyDrag::BodyDrag(const BodyDrag &aForce) :
   Force(aForce),
	_bodyName(_bodyNameProp.getValueStr()),
	_coefficient(_coefficientProp.getValueDbl()),
	_exponent(_exponentProp.getValueDbl())

{
	setNull();
	copyData(aForce);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy this body and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this OpenSim::Body.
 */
Object* BodyDrag::copy() const
{
	BodyDrag *Force = new BodyDrag(*this);
	return(Force);
}
//_____________________________________________________________________________
/**
 * Copy data members from one BodyDrag to another.
 *
 * @param aForce BodyDrag to be copied.
 */
void BodyDrag::copyData(const BodyDrag &aForce)
{
	Force::copyData(aForce);
	_bodyName = aForce._bodyName;
	_coefficient = aForce._coefficient;
	_exponent = aForce._exponent;
}

//_____________________________________________________________________________
/**
 * Set the data members of this BodyDrag to their null values.
 */
void BodyDrag::setNull()
{
	setType("BodyDrag");
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void BodyDrag::setupProperties()
{
	// bodyName
	_bodyNameProp.setName("bodyName");
	_propertySet.append(&_bodyNameProp);

	// coefficient
	_coefficientProp.setName("coefficient");
	_propertySet.append(&_coefficientProp);

	// exponent
	_exponentProp.setName("exponent");
	_propertySet.append(&_exponentProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this BodyDrag.
 */
void BodyDrag::setup(Model& aModel)
{
	string errorMessage;

	// Base class
	Force::setup(aModel);

	// Look up the body and report an error if it is not found 
	if (!aModel.updBodySet().contains(_bodyName)) {
		errorMessage = "Invalid bodyName (" + _bodyName + ") specified in Force " + getName();
		throw (Exception(errorMessage.c_str()));
	}
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
BodyDrag& BodyDrag::operator=(const BodyDrag &aForce)
{
	Force::operator=(aForce);
	copyData(aForce);
	return(*this);
}


//=============================================================================
// COMPUTATION
//=============================================================================

void BodyDrag::computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
	if(_model==NULL) return;		// some minor error checking

	SimTK::Vec3 bodyCoMPosBody, bodyCoMPosGround, bodyCoMVelGround, bodyCoMVelGroundRaisedPower, dragForceGround, dragForceBody, oppVelSign;
	BodySet &bs = _model->updBodySet();										// get body set
	Body &ground = _model->getSimbodyEngine().getGroundBody();				// get ground body
	Body &aBody = bs.get(_bodyName);										// get the body to apply the force to

	aBody.getMassCenter(bodyCoMPosBody);													// get CoM position of body in the BODY coordinate system
	_model->getSimbodyEngine().getPosition(s, aBody, bodyCoMPosBody, bodyCoMPosGround);		// get CoM position of body in the GROUND coordinate system
	_model->getSimbodyEngine().getVelocity(s, aBody, bodyCoMPosBody, bodyCoMVelGround);		// get CoM velocity of body in the GROUND coordinate system

	for (int i=0; i<3;i++)
	{
		if (bodyCoMVelGround[i]>0) oppVelSign[i] = -1;										// get opposite sign of CoM velocity (GROUND coordinate system)
		if (bodyCoMVelGround[i]<0) oppVelSign[i] = 1;
		if (bodyCoMVelGround[i]==0) oppVelSign[i] = 0;

		dragForceGround[i] = oppVelSign[i] * _coefficient * pow(bodyCoMVelGround[i], _exponent);	// calculate drag force in the GROUND coordinate system
	}

	_model->getSimbodyEngine().transform(s, ground, dragForceGround, aBody, dragForceBody);			// transform drag force into the BODY coordinate system


	// Apply drag force to the body
	// ------------------------------
	// applyForceToPoint requires the force application point to be in the inertial (ground) frame
	// and the force vector itself to be in the body frame
	applyForceToPoint(s, aBody, bodyCoMPosGround, dragForceBody, bodyForces);




	// Debuging info
	// --------------
	int deb = 0;
	if (deb)
	{
		cout << "Time = " << s.getTime() << endl;
		cout << aBody.getName() << " CoM position (body frame) = " << bodyCoMPosBody << endl;
		cout << aBody.getName() << " CoM position (ground frame) = " << bodyCoMPosGround << endl;
		cout << aBody.getName() << " CoM velocity (ground frame) = " << bodyCoMVelGround << endl;
		cout << aBody.getName() << " CoM velocity opposite sign (ground frame) = " << oppVelSign << endl;
		cout << aBody.getName() << " CoM velocity^" << _exponent << " (ground frame) = " << bodyCoMVelGroundRaisedPower << endl;
		cout << "Drag coefficient = " << _coefficient << "\tDrag exponent = " << _exponent << endl;
		cout << "dragForce (ground) = " << dragForceGround << endl;
		cout << "dragForce (body frame) = " << dragForceBody << endl;
		system("pause");
	}

	return;
}

/** Potential energy function */
double BodyDrag::computePotentialEnergy(const SimTK::State& s) const
{
	return 0;
}


//=============================================================================
// REPORTING
//=============================================================================
/** 
 * Provide names of the quantities (column labels) of the force value(s) reported
 * 
 */
OpenSim::Array<std::string> BodyDrag::getRecordLabels() const 
{
	OpenSim::Array<std::string> labels("");
	labels.append(getName()+"."+_bodyName+".force.X");
	labels.append(getName()+"."+_bodyName+".force.Y");
	labels.append(getName()+"."+_bodyName+".force.Z");
	return labels;
}
/**
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> BodyDrag::getRecordValues(const SimTK::State& s) const 
{
	OpenSim::Array<double> values(3);

	SimTK::Vec3 bodyCoMPosBody, bodyCoMPosGround, bodyCoMVelGround, bodyCoMVelGroundRaisedPower, dragForceGround, dragForceBody, oppVelSign;
	BodySet &bs = _model->updBodySet();										// get body set
	Body &ground = _model->getSimbodyEngine().getGroundBody();				// get ground body
	Body &aBody = bs.get(_bodyName);										// get the body to apply the force to

	aBody.getMassCenter(bodyCoMPosBody);													// get CoM position of body in the BODY coordinate system
	_model->getSimbodyEngine().getPosition(s, aBody, bodyCoMPosBody, bodyCoMPosGround);		// get CoM position of body in the GROUND coordinate system
	_model->getSimbodyEngine().getVelocity(s, aBody, bodyCoMPosBody, bodyCoMVelGround);		// get CoM velocity of body in the GROUND coordinate system

	for (int i=0; i<3;i++)
	{
		if (bodyCoMVelGround[i]>0) oppVelSign[i] = -1;										// get opposite sign of CoM velocity (GROUND coordinate system)
		if (bodyCoMVelGround[i]<0) oppVelSign[i] = 1;
		if (bodyCoMVelGround[i]==0) oppVelSign[i] = 0;

		dragForceGround[i] = oppVelSign[i] * _coefficient * pow(bodyCoMVelGround[i], _exponent);	// calculate drag force in the GROUND coordinate system
		values.append(dragForceGround[i]);
	}

	return values;
}
