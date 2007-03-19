// LinearSetPoint.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Simulation/Model/AbstractActuator.h>
#include "LinearSetPoint.h"

using namespace std;
using namespace OpenSim;

//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
LinearSetPoint::~LinearSetPoint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
LinearSetPoint::LinearSetPoint(string aBodyA, string aBodyB) :
	SetPoint(aBodyA,aBodyB),
	_knp(_propKNP.getValueDbl()),
	_knv(_propKNV.getValueDbl())
{
	// NULL
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce Force to be copied.
 */
LinearSetPoint::LinearSetPoint(const LinearSetPoint &aContact) :
	SetPoint(aContact),
	_knp(_propKNP.getValueDbl()),
	_knv(_propKNV.getValueDbl())
{
	setNull();

	// STIFFNESS
	setNormalStiffness(aContact.getNormalStiffness());

	// VISCOSITY
	setNormalViscosity(aContact.getNormalViscosity());
}
//_____________________________________________________________________________
/**
 * Copy this actuator and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this actuator.
 */
Object* LinearSetPoint::
copy() const
{
	AbstractActuator *act = new LinearSetPoint(*this);
	return(act);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void LinearSetPoint::
setNull()
{
	setupProperties();
	setType("LinearSetPoint");

	setNumControls(0); setNumStates(0); setNumPseudoStates(3);
	bindPseudoState(0, _pA[0], "px");
	bindPseudoState(1, _pA[1], "py");
	bindPseudoState(2, _pA[2], "pz");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void LinearSetPoint::
setupProperties()
{
	_propKNP.setName("normal_stiffness");
	_propKNP.setValue(0.0);
	_propertySet.append( &_propKNP );

	_propKNV.setName("normal_viscosity");
	_propKNV.setValue(0.0);
	_propertySet.append( &_propKNV );
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
LinearSetPoint& LinearSetPoint::
operator=(const LinearSetPoint &aActuator)
{
	// BASE CLASS
	SetPoint::operator=(aActuator);

	// MEMBER VARIABLES
	_knp = aActuator.getNormalStiffness();
	_knv = aActuator.getNormalViscosity();

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// NORMAL IMPEDANCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the stiffness of the spring in the direction normal to the surface.
 *
 * @return Normal stiffness.
 */
double LinearSetPoint::
getInstantaneousNormalStiffness() const
{
	return(_knp);
}
//_____________________________________________________________________________
/**
 * Get the instantaneous viscosity of the contact element in the direction
 * normal to the surface.
 *
 * @return Normal viscosity.
 */
double LinearSetPoint::
getInstantaneousNormalViscosity() const
{
	return(_knv);
}

//-----------------------------------------------------------------------------
// STIFFNESS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set normal stiffness.
 *
 * @param aKTP Normal stiffness- must be positive.
 */
void LinearSetPoint::
setNormalStiffness(double aKNP)
{
	_knp = aKNP;
	if(_knp<0) _knp=0.0;
}
//_____________________________________________________________________________
/**
 * Get normal stiffness.
 *
 * @return Normal stiffness.
 */
double LinearSetPoint::
getNormalStiffness() const
{
	return(_knp);
}

//-----------------------------------------------------------------------------
// VISCOSITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set normal viscosity.
 *
 * @param aKTV Normal viscosity- must be positive.
 */
void LinearSetPoint::
setNormalViscosity(double aKNV)
{
	_knv = aKNV;
	if(_knv<0) _knv=0.0;
}
//_____________________________________________________________________________
/**
 * Get normal viscosity.
 *
 * @return Normal viscosity.
 */
double LinearSetPoint::
getNormalViscosity() const
{
	return(_knv);
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
void LinearSetPoint::
computeActuation()
{
	// UPDATE SPRING POINTS
	updatePointA();
	updatePointB();
	
	// DISPLACEMENTS AND VELOCITIES
	computeDisplacements();
	computeVelocities();

	// NORMAL FORCE
	double fnp=0.0;
	double fnv=0.0;
	double d = getNormalDistance();
	if(d<=0.0) {
		fnp = -_knp*d;
		fnv = -_knv*getNormalSpeed();
	}
	_fnMag = fnp + fnv;
	Mtx::Multiply(1,3,&_nA[0],fnp,_fnp);
	Mtx::Multiply(1,3,&_nA[0],fnv,_fnv);
	Mtx::Multiply(1,3,&_nA[0],_fnMag,_fn);

	// TANGENTIAL FORCE
	_ftMag = computeTangentialForce(_fnMag,_ft,_dfFric);

	// RESULTANT FORCE
	double fA[3],uA[3];
	Mtx::Add(1,3,_fn,_ft,fA);
	double f = Mtx::Normalize(3,fA,uA);
	Mtx::Multiply(1,3,uA,-1.0,uA);
	if(f==0.0) {
		Mtx::Add(1,3,_rtA,_rnA,uA);
		Mtx::Normalize(3,uA,uA);
	}

	// FORCE AND DIRECTION
	setForce(f);
	setForceDirectionA(uA);

	// SPEED
	double v[3];
	Mtx::Add(1,3,_vnA,_vtA,v);
	_speed = -Mtx::DotProduct(3,uA,v);

	// BODY B
	computeForceDirectionForBodyB();
}


//=============================================================================
// APPLICATION
//=============================================================================


//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
/**
 * Check that this force actuator has a valid set of states.
 */
bool LinearSetPoint::
check() const
{
	bool status = SetPoint::check();

	return(status);
}


//=============================================================================
// XML
//=============================================================================
//-----------------------------------------------------------------------------
// UPDATE FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update this object based on its XML node.
 *
 * This method simply calls SetPoint::updateFromXMLNode() and then calls
 * a few methods in this class to ensure that variable members have been
 * set in a consistent manner.
 */
void LinearSetPoint::
updateFromXMLNode()
{
	SetPoint::updateFromXMLNode();
	setNormalStiffness(_knp);
	setNormalViscosity(_knv);
}	

