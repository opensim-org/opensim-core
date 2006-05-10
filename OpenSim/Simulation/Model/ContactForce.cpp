// ContactForce.cpp
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
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include "ContactForce.h"




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
ContactForce::~ContactForce()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * 
 */
ContactForce::ContactForce(int aBodyA,int aBodyB,int aNYP) :
	Force(aBodyA,aBodyB,0,0,aNYP),
	_nA(_propNormalA.getValueDblArray()),
	_nB(_propNormalB.getValueDblArray())
{
	// NULL
	setNull();
}
//_____________________________________________________________________________
/**
 * Construct a contact force object based on an XML node.
 *
 * @param aElement XML node based on which to construct this object.
 * @param aNYP Number of pseudo-states.
 */
ContactForce::ContactForce(DOMElement *aElement,int aNYP) :
	Force(aElement,0,0,aNYP),
	_nA(_propNormalA.getValueDblArray()),
	_nB(_propNormalB.getValueDblArray())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce Force to be copied.
 */
ContactForce::ContactForce(const ContactForce &aContact) :
	Force(aContact),
	_nA(_propNormalA.getValueDblArray()),
	_nB(_propNormalB.getValueDblArray())
{
	setNull();

	// NORM A
	aContact.getNormalA(&_nA[0]);
	setNormalA(&_nA[0]);

	// NORM B
	aContact.getNormalB(&_nB[0]);
	setNormalB(&_nB[0]);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void ContactForce::
setNull()
{
	//generateProperties();
	setupProperties();

	// TYPE
	setType("ContactForce");

	// NORMAL B
	_nB[0]=0.0;  _nB[1]=0.0;  _nB[2]=0.0;

	// NORMAL A
	_nA[0]=0.0;  _nA[1]=0.0;  _nA[2]=0.0;

	// NORMAL DISPLACMENT
	_rnA[0]=0.0;  _rnA[1]=0.0;  _rnA[2]=0.0;

	// NORMAL DISTANCE
	_rn = 0.0;

	// NORMAL VELOCITY
	_vnA[0]=1.0;  _vnA[1]=0.0;  _vnA[2]=0.0;

	// NORMAL SPEED
	_vn = 0.0;

	// TANGENT
	_tA[0]=1.0;  _tA[1]=0.0;  _tA[2]=0.0;

	// TANGENTIAL DISPLACEMENT
	_rtA[0]=1.0;  _rtA[1]=0.0;  _rtA[2]=0.0;

	// TANGENTIAL DISTANCE
	_rt = 0.0;

	// TANGENTIAL VELOCITY
	_vtA[0]=1.0;  _vtA[1]=0.0;  _vtA[2]=0.0;

	// FORCE GEOMETRY
	_fnMag = 0.0;
	_fnp[0] = _fnp[1] = _fnp[2] = 0.0;
	_fnv[0] = _fnv[1] = _fnv[2] = 0.0;
	_fn[0] = _fn[1] = _fn[2] = 0.0;
	_ftMag = 0.0;
	_ftp[0] = _ftp[1] = _ftp[2] = 0.0;
	_ftv[0] = _ftv[1] = _ftv[2] = 0.0;
	_ft[0] = _ft[1] = _ft[2] = 0.0;

	// FRICTION CORRECTION
	_dfFric[0] = _dfFric[1] = _dfFric[2] = 0.0;

	// POINT VELOCITY FUNTIONS
	_vAFunction = NULL;
	_vBFunction = NULL;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ContactForce::
setupProperties()
{
	double origin[3] = { 0.0, 0.0, 0.0 };

	_propNormalA.setName("normal_A");
	_propNormalA.setValue(3,origin);
	_propertySet.append( &_propNormalA );

	_propNormalB.setName("normal_B");
	_propNormalB.setValue(3,origin);
	_propertySet.append( &_propNormalB );
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
ContactForce& ContactForce::
operator=(const ContactForce &aContact)
{
	// BASE CLASS
	Force::operator=(aContact);

	// NORM A
	aContact.getNormalA(&_nA[0]);

	// NORM B
	aContact.getNormalB(&_nB[0]);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// CONTROLS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the number of controls (0 for ContactForce).
 *
 * Derived classes should override this method.
 *
 * @return Number of controls.
 */
int ContactForce::
getNX() const
{
	return(0);
}
//_____________________________________________________________________________
/**
 * Get the name of a control.\n
 * Valid indices: none
 *
 * Derived classes should override this method.
 *
 * @param aIndex Index of the control whose name is desired.
 * @throws Exception If aIndex is not valid.
 */
const string ContactForce::
getControlName(int aIndex) const
{
	string msg = "ContactForce.getControlName: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no controls";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Get the index of a control of a specified name.\n
 * Valid names: (there are no valid names)
 *
 * Derived classes should override this method.
 *
 * @param aName Name of the control whose index is desired.
 * @return Index of the specified control.
 * @throws Exception If aName is not valid.
 */
int ContactForce::
getControlIndex(const string &aName) const
{
	string msg = "ContactForce.getControlIndex: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no control by the name";
	msg += aName;
	msg += ".";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Set the current values of the controls.
 *
 * Derived classes should override this method.
 *
 * @param aX Array of control values.
 * @throws Exception This actuator has no controls.
 */
void ContactForce::
setControls(const double aX[])
{
	string msg = "ContactForce.setControls: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no controls.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Set the value of a control at a specified index.\n
 * Valid indices: none
 *
 * Derived classes should override this method.
 *
 * @param aIndex Index of the control to be set.
 * @param aValue Value to which to set the control.
 * @throws Exception If aIndex is not valid.
 */
void ContactForce::
setControl(int aIndex,double aValue)
{
	string msg = "ContactForce.setControl: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no controls.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Set the value of a control of a specified name.\n
 * Valid names: (there are no valid names)
 *
 * Derived classes should override this method.
 *
 * @param aName Name of the control to be set.
 * @param aValue Value to which to set the control.
 * @throws Exception If aName is not valid.
 */
void ContactForce::
setControl(const string &aName,double aValue)
{
	string msg = "ContactForce.setControl: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no controls.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Get the current values of the controls.
 *
 * Derived classes should override this method.
 *
 * @param aX Array of control values.
 * @throws Exception There are no controls.
 */
void ContactForce::
getControls(double rX[]) const
{
	string msg = "ContactForce.getControls: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no controls.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Get the value of a control at a specified index.\n
 * Valid indices: none
 *
 * Derived classes should override this method.
 *
 * @param aIndex Index of the desired control.
 * @return Value of the desired control.
 * @throws Exception If aIndex is not valid.
 */
double ContactForce::
getControl(int aIndex) const
{
	string msg = "ContactForce.setControl: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no controls.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Get the value of a control of a specified name.\n
 * Valid names: (there are no valid names)
 *
 * Derived classes should override this method.
 *
 * @param aName Name of the desired control.
 * @return Value of the desired control.
 * @throws Exception If aName is not valid.
 */
double ContactForce::
getControl(const string &aName) const
{
	string msg = "ContactForce.setControl: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no controls.";
	throw( Exception(msg,__FILE__,__LINE__) );
}

//-----------------------------------------------------------------------------
// NORMAL B
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the direction of the surface normal of BodyB at the location where
 * the contact force is applied.
 *
 * When this method is used to set the surface normal for BodyB, the last
 * surface normal set for BodyA is ignored.
 *
 * @param aNormal Direction of surface normal in body-local coordinates.
 * @see setNormalA()
 */
void ContactForce::
setNormalB(const double aNormal[3])
{
	double mag = Mtx::Normalize(3,aNormal,&_nB[0]);
	if(mag<=rdMath::ZERO) _nB[0] = _nB[1] = _nB[2] = 0.0;
}
//_____________________________________________________________________________
/**
 * Get the direction of the surface normal of BodyB at the location where
 * the contact force is applied.
 *
 * @param rNormal Surface normal of BodyB.
 */
void ContactForce::
getNormalB(double rNormal[3]) const
{
	rNormal[0] = _nB[0];
	rNormal[1] = _nB[1];
	rNormal[2] = _nB[2];
}

//-----------------------------------------------------------------------------
// NORMAL A
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the direction of the surface normal of BodyA at the location where
 * the contact force is applied.
 *
 * When this method is used to set the surface normal for BodyA, the last
 * surface normal set for BodyB is ignored.
 *
 * @param aNormal Direction of surface normal in body-local coordinates.
 * @see setNormalB()
 */
void ContactForce::
setNormalA(const double aNormal[3])
{
	double mag = Mtx::Normalize(3,aNormal,&_nA[0]);
	if(mag<=rdMath::ZERO) _nA[0] = _nA[1] = _nA[2] = 0.0;
}
//_____________________________________________________________________________
/**
 * Get the direction of the surface normal of BodyA at the location where
 * the contact force is applied.
 *
 * @param rNormal Surface normal of BodyA.
 */
void ContactForce::
getNormalA(double rNormal[3]) const
{
	rNormal[0] = _nA[0];
	rNormal[1] = _nA[1];
	rNormal[2] = _nA[2];
}

//-----------------------------------------------------------------------------
// NORMAL DISPLACEMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the normal component of the displacement vector directed from
 * PointA to PointB expressed in the local frame of BodyA.
 *
 * ComputeDisplacementVectors() should be called prior to calling this
 * method to ensure that the returned result is valid.
 *
 * @param rDisplacment Normal displacement vector.
 */
void ContactForce::
getNormalDisplacement(double rDisplacment[3]) const
{
	rDisplacment[0] = _rnA[0];
	rDisplacment[1] = _rnA[1];
	rDisplacment[2] = _rnA[2];
}

//-----------------------------------------------------------------------------
// NORMAL DISTANCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the magnitude of the normal displacement from PointA to PointB.
 * A negative magnitude indicates that bodies A and B are penetrating.
 *
 * ComputeDisplacementVectors() should be called prior to calling this
 * method to ensure that the returned result is valid.
 *
 * @return Normal distance  A positive value indicates penetration.
 */
double ContactForce::
getNormalDistance() const
{
	return(_rn);
}

//-----------------------------------------------------------------------------
// NORMAL VELOCITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the normal component of the velocity of PointB relative to PointA.
 *
 * computeActuation() should be called prior to calling this
 * method to ensure that the returned result is valid.
 *
 * @param rVelocity Normal velocity vector.
 */
void ContactForce::
getNormalVelocity(double rVelocity[3]) const
{
	rVelocity[0] = _rnA[0];
	rVelocity[1] = _rnA[1];
	rVelocity[2] = _rnA[2];
}

//-----------------------------------------------------------------------------
// NORMAL SPEED
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the magnitude of the normal velocity of PointB relative to PointA.
 * A negative value indicates that the bodies are getting closer together.
 *
 * computeActuation() should be called prior to calling this
 * method to ensure that the returned result is valid.
 *
 * @return Normal distance  A positive value indicates penetration.
 */
double ContactForce::
getNormalSpeed() const
{
	return(_vn);
}


//-----------------------------------------------------------------------------
// TANGENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the tangential component of the displacement of PointB relative to
 * to PointA.
 *
 * computeActuation() should prior to a call to this method for the
 * returned results to be valid.
 *
 * @param rTangent expressed as a unit vector in the local frame of BodyA.
 */
void ContactForce::
getTangent(double rTangent[3]) const
{
	rTangent[0] = _tA[0];
	rTangent[1] = _tA[1];
	rTangent[2] = _tA[2];
}

//-----------------------------------------------------------------------------
// TANGENTIAL DISPLACEMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the tangential component of the displacement vector directed from
 * PointA to PointB expressed in the local frame of BodyA.
 *
 * computeActuation() should be called prior to calling this
 * method to ensure that the returned result is valid.
 *
 * @param rDisplacement Tangential displacement vector.
 */
void ContactForce::
getTangentialDisplacement(double rDisplacement[3]) const
{
	rDisplacement[0] = _rtA[0];
	rDisplacement[1] = _rtA[1];
	rDisplacement[2] = _rtA[2];
}

//-----------------------------------------------------------------------------
// TANGENTIAL DISTANCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the magnitude of the tangential displacement from PointA to PointB.
 *
 * computeActuation() should be called prior to calling this
 * method to ensure that the returned result is valid.
 *
 * @return Tangential displacement magnitude.  The returned value is
 * always positive.
 */
double ContactForce::
getTangentialDistance() const
{
	return(_rt);
}

//-----------------------------------------------------------------------------
// TANGENTIAL VELOCITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the tangential component of the velocity of PointB relative to PointA
 * expressed in the local frame of BodyA.
 *
 * computeActuation() should be called prior to calling this
 * method to ensure that the returned result is valid.
 *
 * @param rVelocity Tangential velocity vector.
 */
void ContactForce::
getTangentialVelocity(double rVelocity[3]) const
{
	rVelocity[0] = _vtA[0];
	rVelocity[1] = _vtA[1];
	rVelocity[2] = _vtA[2];
}

//-----------------------------------------------------------------------------
// FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the normal force acting on BodyB expressed in the local frame
 * of BodyA.
 *
 * computeActuation() should be called prior to calling this
 * method to ensure that the returned result is valid.
 *
 * @param rFP Elastic normal force.
 * @param rFV Viscous normal force.
 * @param rF Total normal force.
 */
void ContactForce::
getNormalForce(double rFP[3],double rFV[3],double rF[3]) const
{
	Mtx::Assign(1,3,_fnp,rFP);
	Mtx::Assign(1,3,_fnv,rFV);
	Mtx::Assign(1,3,_fn,rF);
}
//_____________________________________________________________________________
/**
 * Get the tangential force acting on BodyB expressed in the local frame
 * of BodyA.
 *
 * computeActuation() should be called prior to calling this
 * method to ensure that the returned result is valid.
 *
 * @param rFP Elastic normal force NOT corrected to enforce friction
 * constraints.
 * @param rFV Viscous normal force NOT corrected to enforce friction
 * constraints.
 * @param rF Total normal force corrected to enforce friction constraints.
 * This force is the actual force applied to BodyB.
 */
void ContactForce::
getTangentialForce(double rFP[3],double rFV[3],double rF[3]) const
{
	Mtx::Assign(1,3,_ftp,rFP);
	Mtx::Assign(1,3,_ftv,rFV);
	Mtx::Assign(1,3,_ft,rF);
}

//-----------------------------------------------------------------------------
// FRICTION CORRECTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the coorection made in the spring force in order to enforce
 * friction constaints.  The correction is the change made to the force
 * applied to BodyB expressed in the local frame of BodyA.
 *
 * Friction corrections normally do not involve corrections to the normal
 * force, so the correction will likely have zero magnitude in the direction
 * of the normal force.
 *
 * @param rDF Change made to the contact force applied to BodyA expressed in
 * the local frame of BodyA. 
 */
void ContactForce::
getFrictionCorrection(double rDF[3]) const
{
	rDF[0] = _dfFric[0];
	rDF[1] = _dfFric[1];
	rDF[2] = _dfFric[2];
}

//-----------------------------------------------------------------------------
// VELOCITY FUNCTIONS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the vector function to read in values for the velocity of Point A.
 *
 * @param aVectorFunction Vector function containing  x, y, and z values of
 * linear velocity of Point A (in the inertial frame) as a function fo time.
 */
void ContactForce::
setVelPointAFunction(VectorFunction* aVectorFunction)
{
	_vAFunction = aVectorFunction;
}
//_____________________________________________________________________________
/**
 * Get the vector function to read in values for the velocity of Point A.
 *
 * @param rVectorFunction Vector function containing  x, y, and z values of
 * linear velocity of Point A (in the inertial frame) as a function fo time.
 */
const VectorFunction* ContactForce::
getVelPointAFunction() const
{
	return(_vAFunction);
}
//_____________________________________________________________________________
/**
 * Set the vector function to read in values for the velocity of Point B.
 *
 * @param aVectorFunction Vector function containing  x, y, and z values of
 * linear velocity of Point B (in the inertial frame) as a function fo time.
 */
void ContactForce::
setVelPointBFunction(VectorFunction* aVectorFunction)
{
	_vBFunction = aVectorFunction;
}
//_____________________________________________________________________________
/**
 * Get the vector function to read in values for the velocity of Point B.
 *
 * @param rVectorFunction Vector function containing  x, y, and z values of
 * linear velocity of Point B (in the inertial frame) as a function fo time.
 */
const VectorFunction* ContactForce::
getVelPointBFunction() const
{
	return(_vBFunction);
}

//-----------------------------------------------------------------------------
// TANGENTIAL IMPEDANCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the stiffness of the spring in the plane tangent to the surface
 * normal.
 *
 * Derived classes should override this method.
 *
 * @return Tangent stiffness. (Always zero unless this method is overridden.)
 */
double ContactForce::
getInstantaneousTangentialStiffness() const
{
	return(0.0);
}
//_____________________________________________________________________________
/**
 * Get the instantaneous viscosity of the contact element in the plane
 * tangent to the surface normal.
 *
 * Derived classes should override this method.
 *
 * @return Tangent viscosity. (Always zero unless this method is overridden.)
 */
double ContactForce::
getInstantaneousTangentialViscosity() const
{
	return(0.0);
}

//-----------------------------------------------------------------------------
// NORMAL IMPEDANCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the stiffness of the spring in the direction normal to the surface.
 *
 * Derived classes should override this method.
 *
 * @return Normal stiffness. (Always 0.0 unless this method is overriden.)
 */
double ContactForce::
getInstantaneousNormalStiffness() const
{
	return(0.0);
}
//_____________________________________________________________________________
/**
 * Get the instantaneous viscosity of the contact element in the direction
 * normal to the surface.
 *
 * Derived classes should override this method.
 *
 * @return Normal viscosity. (Always 0.0 unless this method is overriden.)
 */
double ContactForce::
getInstantaneousNormalViscosity() const
{
	return(0.0);
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Update the pseudostates.
 *
 * Derived classes should override this method.
 *
 * Pseudostates are quantities that are not integrated but that depend on
 * the time history of an integration (e.g., spring set points).
 *
 * This method assumes that the model states are up-to-date (i.e., that
 * model->set(t,x,y) or a similar method has been called).
 *
 */
void ContactForce::
updatePseudoStates()
{
}

//_____________________________________________________________________________
/**
 * Compute the normal and tangential displacements of PointB relative
 * to PointA.
 *
 * @see computeLineOfActionComponents()
 */
void ContactForce::
computeDisplacements()
{
	if(_model==NULL) return;

	// DISPLACEMENTS
	computeLineOfActionComponents(_rnA,_rtA);

	// NORMAL
	_rn = Mtx::DotProduct(3,_rnA,&_nA[0]);

	// TANGENTIAL
	_rt = Mtx::Normalize(3,_rtA,_tA);
}

//_____________________________________________________________________________
/**
 * Compute the normal and tangential velocities of PointB relative to PointA.
 *
 * @see computeLineOfActionComponents()
 */
void ContactForce::
computeVelocities()
{
	if(_model==NULL) return;
	double time = _model->getTime()*_model->getTimeNormConstant();

	// VELOCITY
	double va[3],vb[3],v[3];
	if(_vAFunction == NULL){
		_model->getVelocity(_bA,&_pA[0],va);
	} else {
		_vAFunction->evaluate(&time,va);
	}
	if(_vBFunction == NULL){
		_model->getVelocity(_bB,&_pB[0],vb);
	} else {
		_vBFunction->evaluate(&time,vb);
	}
	Mtx::Subtract(1,3,vb,va,v);
	_model->transform(_model->getGroundID(),v,_bA,v);

	// NORMAL
	_vn = Mtx::DotProduct(3,v,&_nA[0]);
	Mtx::Multiply(1,3,&_nA[0],_vn,_vnA);

	// TANGENTIAL
	Mtx::Subtract(1,3,v,_vnA,_vtA);
}


//=============================================================================
// APPLICATION
//=============================================================================


//=============================================================================
// UTILITY
//=============================================================================
//-----------------------------------------------------------------------------
// LINE OF ACTION, NORMAL AND TANGENTIAL COMPONENTS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the normal and tangential components of the line of action of
 * the force expressed in the local frame of BodyA.
 * The normal component is the projection of the line of action onto the
 * normal of BodyA:
 *
 *		normal = Dot(n,LOA) * nA.
 *
 * The tangential component is the vector difference between the line of
 * action and its normal component:
 *
 *		tangential = LOA - normalLOA.
 *
 * @param rNormal Normal component of line of action expressed
 * in the local frame of BodyA.
 * @param rTangential Tangential component of line of action expressed
 * in the local frame of BodyA.
 */
void ContactForce::
computeLineOfActionComponents(double rNormal[3],
	double rTangential[3]) const
{
	// LINE OF ACTION
	double r[3];
	computeLineOfAction(r);
	_model->transform(_model->getGroundID(),r,_bA,r);

	// NORMAL COMPONENT
	double rn = Mtx::DotProduct(3,r,&_nA[0]);
	Mtx::Multiply(1,3,&_nA[0],rn,rNormal);

	// TANGENTIAL COMPONENT
	Mtx::Subtract(1,3,r,rNormal,rTangential);
}


//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
/**
 * Check that this actuator is valid.
 */
bool ContactForce::
check() const
{
	Force::check();

	// NORMAL A
	if(Mtx::Magnitude(3,&_nA[0])<rdMath::ZERO) {
		printf("ContactForce.check: WARN- BodyA of contact %s has an invalid ",
			getName().c_str());
		printf("surface normal:\n\t  nA = %lf %lf %lf.\n",_nA[0],_nA[1],_nA[2]);
	}

	// NORMAL B
	if(Mtx::Magnitude(3,&_nB[0])<rdMath::ZERO) {
		printf("ContactForce.check: WARN- BodyB of contact %s has an invalid ",
			getName().c_str());
		printf("surface normal:\n\t  nB = %lf %lf %lf.\n",_nB[0],_nB[1],_nB[2]);
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
void ContactForce::
updateFromXMLNode()
{
	Force::updateFromXMLNode();
	setNormalA(&_nA[0]);
	setNormalB(&_nB[0]);
}	
