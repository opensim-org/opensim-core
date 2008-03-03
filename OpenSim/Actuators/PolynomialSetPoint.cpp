// PolynomialSetPoint.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/PropertyDbl.h>
#include "PolynomialSetPoint.h"




using namespace OpenSim;
using namespace std;


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
PolynomialSetPoint::~PolynomialSetPoint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PolynomialSetPoint::PolynomialSetPoint(string aBodyA,string aBodyB) :
	SetPoint(aBodyA,aBodyB),
	_kNP(_propKNP.getValueDbl()),
	_kNV(_propKNV.getValueDbl()),
	_powNP(_propPowerNP.getValueDbl()),
	_powNV(_propPowerNV.getValueDbl())
{
	// NULL
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aContact Contact to be copied.
 */
PolynomialSetPoint::PolynomialSetPoint(const PolynomialSetPoint &aContact) :
	SetPoint(aContact),
	_kNP(_propKNP.getValueDbl()),
	_kNV(_propKNV.getValueDbl()),
	_powNP(_propPowerNP.getValueDbl()),
	_powNV(_propPowerNV.getValueDbl())
{
	setNull();

	// STIFFNESS
	setNormalStiffnessConstant(aContact.getNormalStiffnessConstant());

	// VISCOSITY
	setNormalViscosityConstant(aContact.getNormalViscosityConstant());

	// POWERS
	setStiffnessPower(aContact.getStiffnessPower());
	setViscosityPower(aContact.getViscosityPower());
}
//_____________________________________________________________________________
/**
 * Copy this actuator and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this actuator.
 */
Object* PolynomialSetPoint::
copy() const
{
	AbstractActuator *act = new PolynomialSetPoint(*this);
	return(act);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void PolynomialSetPoint::
setNull()
{
	setupProperties();
	setType("PolynomialSetPoint");

	setNumControls(0); setNumStates(0); setNumPseudoStates(3);
	bindPseudoState(0, _pA[0], "px");
	bindPseudoState(1, _pA[1], "py");
	bindPseudoState(2, _pA[2], "pz");

	// MEMBER VARIABLES
	_kNP = 0.0;
	_kNV = 0.0;
	_powNP = 1.0;
	_powNV = 1.0;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PolynomialSetPoint::
setupProperties()
{
	_propKNP.setName("normal_stiffness");
	_propKNP.setValue(0.0);
	_propertySet.append( &_propKNP );

	_propKNV.setName("normal_viscosity");
	_propKNV.setValue(0.0);
	_propertySet.append( &_propKNV );

	_propPowerNP.setName("stiffness_power");
	_propPowerNP.setValue(0.0);
	_propertySet.append( &_propPowerNP );

	_propPowerNV.setName("viscosity_power");
	_propPowerNV.setValue(0.0);
	_propertySet.append( &_propPowerNV );
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
 * @param aActuator Actuator to which to set this actuator equal.
 * @return Reference to this object.
 */
PolynomialSetPoint& PolynomialSetPoint::
operator=(const PolynomialSetPoint &aActuator)
{
	// BASE CLASS
	SetPoint::operator=(aActuator);

	// MEMBER VARIABLES
	_kNP = aActuator.getNormalStiffnessConstant();
	_kNV = aActuator.getNormalViscosityConstant();
	_powNP = aActuator.getStiffnessPower();
	_powNV = aActuator.getViscosityPower();

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
double PolynomialSetPoint::
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
double PolynomialSetPoint::
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
void PolynomialSetPoint::
setNormalStiffnessConstant(double aKNP)
{
	_kNP = aKNP;
	if(_kNP<0) _kNP=0.0;
}
//_____________________________________________________________________________
/**
 * Get normal stiffness.
 *
 * @return Normal stiffness.
 */
double PolynomialSetPoint::
getNormalStiffnessConstant() const
{
	return(_kNP);
}

//-----------------------------------------------------------------------------
// VISCOSITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set normal viscosity.
 *
 * @param aKNV Normal viscosity- must be positive.
 */
void PolynomialSetPoint::
setNormalViscosityConstant(double aKNV)
{
	_kNV = aKNV;
	if(_kNV<0) _kNV=0.0;
}
//_____________________________________________________________________________
/**
 * Get normal viscosity.
 *
 * @return Normal viscosity.
 */
double PolynomialSetPoint::
getNormalViscosityConstant() const
{
	return(_kNV);
}

//-----------------------------------------------------------------------------
// STIFFNESS POWER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the power to which the normal displacement of the spring is raised.
 *
 * @param aPower Power to which the normal displacement of the spring is
 * raised.
 */
void PolynomialSetPoint::
setStiffnessPower(double aPower)
{
	_powNP = aPower;
}
//_____________________________________________________________________________
/**
 * Get the power to which the normal displacement of the spring is raised.
 *
 * @return Power to which the normal displacement of the spring is raised.
 */
double PolynomialSetPoint::
getStiffnessPower() const
{
	return(_powNP);
}

//-----------------------------------------------------------------------------
// VISCOSITY POWER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the power to which the normal velocity of the spring is raised.
 *
 * @param aPower Power to which the normal velocity of the spring is raised.
 */
void PolynomialSetPoint::
setViscosityPower(double aPower)
{
	_powNV = aPower;
}
//_____________________________________________________________________________
/**
 * Get the power to which the normal velocity of the spring is raised.
 *
 * @return Power to which the normal velocity of the spring is raised.
 */
double PolynomialSetPoint::
getViscosityPower() const
{
	return(_powNV);
}



//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the spring forces.
 *
 * @todo The instantaneous viscosities and stiffnesses are no longer correct
 * because of the addition of the "c" coefficient.
 */
void PolynomialSetPoint::
computeActuation()
{
	// UPDATE SPRING POINTS
	updatePointA();
	updatePointB();
	
	// DISPLACEMENTS AND VELOCITIES
	computeDisplacements();
	computeVelocities();

	// NORMAL FORCE
	// Stiffness term
	double fnp = 0.0;
	double d = getNormalDistance();
	if(d<0.0) {
		d = fabs(d);
		fnp = _kNP * pow(d,_powNP);
	}
	// Viscosity term
	double fnv = 0.0;
	double fnvTmp = 0.0;
	double s = getNormalSpeed();
	if((fnp>0.0)&&(_powNV>0.0)) {
		fnvTmp = _kNV*pow(fabs(s),_powNV);
		fnvTmp = rdMath::CopySign(fnvTmp,-s);
		double c = rdMath::SigmaUp(4.0,20.0,fnp);
		fnv = c * fnvTmp;
		//cout<<"fnp,c,fnv="<<fnp<<", "<<c<<", "<<fnv<<endl;
	}
	_fnMag = fnp + fnv;
	if(_fnMag<0.0) _fnMag = 0.0;
	_fnp=fnp*_nA;
	_fnv=fnv*_nA;
	_fn=_fnMag*_nA;

	// INSTANTANEOUS NORMAL STIFFNESS
	_knp = 0.0;
	if(fnp!=0.0) {
		_knp = _kNP*_powNP*pow(d,_powNP-1.0)*(1.0 + fnvTmp);
	}

	// INSTANTANEOUS NORMAL VISCOSITY
	_knv = 0.0;
	if(fnp!=0.0) {
		_knv = fnp*_kNV*_powNV*pow(fabs(s),_powNV-1.0);
	}

	// TANGENTIAL FORCE
	_ftMag = computeTangentialForce(_fnMag,_ft,_dfFric);

	// RESULTANT FORCE
	SimTK::Vec3 fA,uA;
	fA=_fn+_ft;
	double f = Mtx::Normalize(3,fA,uA);
	uA*= -1.0;
	if(f==0.0) {
		uA=_rtA+_rnA;
		Mtx::Normalize(3,uA,uA);
	}

	// FORCE AND DIRECTION
	setForce(f);
	setForceDirectionA(uA);

	// SPEED
	SimTK::Vec3 v=_vnA+_vtA;
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
bool PolynomialSetPoint::
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
 * This method simply calls Object::updateFromXMLNode() and then calls
 * a few methods in this class to ensure that variable members have been
 * set in a consistent manner.
 */
void PolynomialSetPoint::
updateFromXMLNode()
{
	SetPoint::updateFromXMLNode();

	setNormalStiffnessConstant(_kNP);
	setNormalViscosityConstant(_kNV);
	setStiffnessPower(_powNP);
	setViscosityPower(_powNV);
}	

