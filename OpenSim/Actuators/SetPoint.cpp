// SetPoint.cpp
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
#include "SetPoint.h"



using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SetPoint::~SetPoint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 * SetPoint has three pseudo-states, which are the coordinates of contact
 * PointA.  Derived classes should be aware of this.
 *
 * @param aBodyA Contact BodyA.
 * @param aBodyB Contact BodyB.
 */
SetPoint::SetPoint(string aBodyA,string aBodyB) :
	ContactForce(aBodyA,aBodyB),
	_ktp(_propKTP.getValueDbl()),
	_ktv(_propKTV.getValueDbl()),
	_mu(_propMU.getValueDbl())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce Force to be copied.
 */
SetPoint::SetPoint(const SetPoint &aActuator) :
	ContactForce(aActuator),
	_ktp(_propKTP.getValueDbl()),
	_ktv(_propKTV.getValueDbl()),
	_mu(_propMU.getValueDbl())
{
	setNull();

	// STIFFNESS
	setTangentialStiffness(aActuator.getTangentialStiffness());

	// VISCOSITY
	setTangentialViscosity(aActuator.getTangentialViscosity());

	// FRICTION
	setFriction(aActuator.getFriction());
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void SetPoint::
setNull()
{
	setType("SetPoint");
	setupProperties();

	setNumControls(0); setNumStates(0); setNumPseudoStates(3);
	bindPseudoState(0, _pA[0], "px");
	bindPseudoState(1, _pA[1], "py");
	bindPseudoState(2, _pA[2], "pz");

	// FRICTION
	_mu = 0.0;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SetPoint::
setupProperties()
{
	_propKTP.setName("tangential_stiffness");
	_propKTP.setValue(0.0);
	_propertySet.append( &_propKTP );

	_propKTV.setName("tangential_viscosity");
	_propKTV.setValue(0.0);
	_propertySet.append( &_propKTV );

	_propMU.setName("coefficient_of_friction");
	_propMU.setValue(0.0);
	_propertySet.append( &_propMU );
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
SetPoint& SetPoint::
operator=(const SetPoint &aSetPoint)
{
	// BASE CLASS
	ContactForce::operator=(aSetPoint);

	// MEMBER VARIABLES
	_mu = aSetPoint.getFriction();
	_ktp = aSetPoint.getTangentialStiffness();
	_ktv = aSetPoint.getTangentialViscosity();

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// TANGENTIAL IMPEDANCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the stiffness of the spring in the plane tangent to the surface
 * normal.
 *
 * @return Tangent stiffness.
 */
double SetPoint::
getInstantaneousTangentialStiffness() const
{
	return _ktp;
}
//_____________________________________________________________________________
/**
 * Get the instantaneous viscosity of the contact element in the plane
 * tangent to the surface normal.
 *
 * @return Tangent viscosity.
 */
double SetPoint::
getInstantaneousTangentialViscosity() const
{
	return _ktv;
}

//-----------------------------------------------------------------------------
// STIFFNESS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set tangential stiffness.
 *
 * @param aKTP Tangential stiffness- must be positive.
 */
void SetPoint::
setTangentialStiffness(double aKTP)
{
	_ktp = aKTP;
	if(_ktp<0.0) _ktp=0.0;
}
//_____________________________________________________________________________
/**
 * Get tangential stiffness.
 *
 * @return Tangential stiffness.
 */
double SetPoint::
getTangentialStiffness() const
{
	//if(_force==0.0) return(0.0);  This line shouldn't be here? -- Clay
	return(_ktp);
}

//-----------------------------------------------------------------------------
// VISCOSITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set tangential viscosity.
 *
 * @param aKTV Tangential viscosity- must be positive.
 */
void SetPoint::
setTangentialViscosity(double aKTV)
{
	_ktv = aKTV;
	if(_ktv<0.0) _ktv=0.0;
}
//_____________________________________________________________________________
/**
 * Get tangential viscosity.
 *
 * @return Tangential viscosity.
 */
double SetPoint::
getTangentialViscosity() const
{
	//if(_force==0.0) return(0.0);  This line shouldn't be here? -- Clay
	return(_ktv);
}

//-----------------------------------------------------------------------------
// FRICTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set friction coefficient.
 *
 * Sliding occurs when the tangential force exceeds the product of the
 * frictional coefficient and the normal force. 
 *
 * @param aMU Coefficient of Friction.  Must be positive.  If aMU is negative,
 * the fricition coefficient is set to 0.0.  A fricition coefficient greater
 * than 1.0 corresponds to stiction.
 */
void SetPoint::
setFriction(double aMU)
{
	_mu = aMU;
	if(_mu<0.0) _mu=0.0;
}
//_____________________________________________________________________________
/**
 * Get friction coefficient.
 *
 * @return Tangent viscosity.
 */
double SetPoint::
getFriction() const
{
	return(_mu);
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Update the pseudostates.
 *
 * Pseudostates are quantities that are not integrated but that depend on
 * the time history of an integration (e.g., spring set points).
 *
 * This method assumes that the model states are up-to-date (i.e., that
 * model->set(t,x,y) or a similar method has been called).
 *
 * SetPoint has three pseudostates that are the coordinates of the contact
 * point on BodyA.
 *
 * @see SetPoint::setPointA()
 */
void SetPoint::
updatePseudoStates()
{
	// COMPUTE ACTUATION
	computeActuation();

	// COMPUTE A NEW SET POINT
	double pA[3];
	bool changed = computeNewSetPoint(_fnMag,pA);
	if(changed) setPointA(pA);
}


//=============================================================================
// APPLICATION
//=============================================================================


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the tangential contact force exerted on BodyB expressed in the
 * local frame of BodyA.  Note that because of damping, the tangential force
 * vector does not necessarily lie along the line directed from PointA to
 * PointB.
 *
 * This method returns the correction made to the tangential force
 * in order to maintain friction constraints.
 *
 * NOTE that this method neither sets the tangential force or friction
 * correction in base class Force (Force::_ftA and Force::_dfFric).
 * It is left to derived classes to set these member variables.
 *
 * This method does set the values of the elastic and viscous components
 * of the tangent force.  Corrections are not made to these quantities
 * to enforce frictional constraints.  Only the resultant tangential
 * force has been changed to satisfy friction constraints.
 *
 * @param aNormalForce Normal force currently exerted by the contact force.
 * The tangential force is not allowed to exceed the coefficient of
 * friction times the normal force.
 * @param rFT Total tangential force exerted on BodyA expressed in the
 * local frame of BodyA.
 * @param rDFFric Change in the force exerted on BodyA to enforce
 * friction constraints expressed in the local frame of BodyA
 * (i.e., rDFFric = rTangentForce - TangentForce(uncorrected), where
 * |rTangentForce| = mu * aNormalForce).  If no correction is needed
 * rDFFric = {0,0,0}.
 * @return Magnitude of the tangential force.
 */
double SetPoint::
computeTangentialForce(double aNormalForce,double rFT[3],
	double rDFFric[3])
{
	// ELASTIC FORCE
	double f = -_ktp * getTangentialDistance();
	Mtx::Multiply(1,3,_tA,f,_ftp);

	// VISCOUS FORCE
	Mtx::Multiply(1,3,_vtA,-_ktv,_ftv);

	// RESULTS
	Mtx::Add(1,3,_ftv,_ftp,rFT);
	rDFFric[0] = rDFFric[1] = rDFFric[2] = 0.0;

	// LIMIT
	// ut = unit vector in the direction of the tangent force
	// df = change in tangential force to enforce friction constraints
	double df;
	double ut[3];
	double limit = _mu * fabs(aNormalForce);
	f = Mtx::Normalize(3,rFT,ut);
	if(f>limit) {
		df = limit - f;
		f = limit;
		Mtx::Multiply(1,3,ut,f,rFT);
		Mtx::Multiply(1,3,ut,df,rDFFric);
	}

	return(f);
}
//_____________________________________________________________________________
/**
 * Compute the setpoint (PointA) so that it is consistent with the
 * displacement of PointB amd the elastic properties of the tangential spring.
 * The viscous properties are not considered in this calculation.
 *
 * @param aNormalForce Normal force.
 * @param rSetPoint Setpoint (PointA) in the local frame of BodyA.
 * @return True if a new set point was computed, False otherwise.
 */
bool SetPoint::
computeNewSetPoint(double aNormalForce,double rSetPoint[3]) const
{
	if(rSetPoint==NULL) return(false);

	// CHECK FOR ZERO STIFFNESS
	if(_ktp==0.0) {
		Mtx::Add(1,3,&_pA[0],_rtA,rSetPoint);
		return(true);
	}

	// FORCE LIMIT
	double limit = _mu * fabs(aNormalForce);

	// ELASTIC FORCE
	double f = _ktp * getTangentialDistance();
	if(f<=limit) return(false);
	f = limit;

	// SETPOINT
	// dt = change in tangential distance
	// _tA = tangent displacement unit vector
	// _dtA = vector change needed to PointA
	// _pA = PointA
	double dt,dtA[3];
	dt = getTangentialDistance() - f/_ktp;
	Mtx::Multiply(1,3,_tA,dt,dtA);
	Mtx::Add(1,3,&_pA[0],dtA,rSetPoint);
	return(true);
}


//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
/**
 * Check that this force actuator has a valid set of states.
 */
bool SetPoint::
check() const
{
	bool status = ContactForce::check();
	if(!status) return(false);

	// STIFFNESS
	if(_ktp<=0.0) {
		printf("SetPoint.check: WARN- tangential stiffness of %s ",getName().c_str());
		printf("is not positive: ktp = %lf\n",_ktp);
	}
	
	// VISCOSITY
	if(_ktv<=0.0) {
		printf("SetPoint.check: WARN- tangential viscosity of %s ",
			getName().c_str());
		printf("is not positive: ktv = %lf\n",_ktv);
	}

	// COEFFICIENT OF FRICTION
	if((_mu<0.0)||(_mu>1.0)) {
		printf("SetPoint.check: WARN- coeficient of friction of %s ",
			getName().c_str());
		printf("is outside normal range: mu = %lf\n",_mu);
	}

	return(true);
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
 * This method simply calls Object::updateFromXMLNode() and then calls
 * a few methods in this class to ensure that variable members have been
 * set in a consistent manner.
 */
void SetPoint::
updateFromXMLNode()
{
	ContactForce::updateFromXMLNode();
	setTangentialStiffness(_ktp);
	setTangentialViscosity(_ktv);
	setFriction(_mu);
}	
