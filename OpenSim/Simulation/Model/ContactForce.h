#ifndef _ContactForce_h_
#define _ContactForce_h_
// ContactForce.h
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


#include "Force.h"


//=============================================================================
//=============================================================================
/**
 * A abstract class that supports the application of contact force between
 * two bodies, BodyA and BodyB.  Actuators of this type have no states
 * and no controls.  That is, contact is assumed to be entirely passive
 * and depends only on the states of a model or models.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class RDSIMULATION_API ContactForce : public Force
{

//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Surface normal on BodyA expressed in the BodyA frame. */
	PropertyDblArray _propNormalA;
	/** Surface normal on BodyB expressed in the BodyB frame. */
	PropertyDblArray _propNormalB;

	// REFERENCES
	Array<double> &_nA;
	Array<double> &_nB;


	/** Normal displacement vector expressed in the BodyA frame. */
	double _rnA[3];
	/** Normal distance from PointA to PointB. */
	double _rn;
	/** Normal velocity of PointB relative to PointA expressed in the BodyA
	frame. */
	double _vnA[3];
	/** Normal speed of PointB relative to PointA. */
	double _vn;
	/** Tangent displacement unit vector expressed in the BodyA frame. */
	double _tA[3];
	/** Tangential displacement vector expressed in the BodyA frame. */
	double _rtA[3];
	/** Tangential distance from PointA to PointB. */
	double _rt;
	/** Tangential velocity of PointB relative to PointA expressed in the
	BodyA frame. */
	double _vtA[3];
	// FORCE GEOMETRY
	/** Total normal force magnitude. */
	double _fnMag;
	/** Elastic normal force applied to BodyB expressed in the local frame
	of BodyA. */
	double _fnp[3];
	/** Viscous normal force applied to BodyB expressed in the local frame
	of BodyA. */
	double _fnv[3];
	/** Total ormal force applied to BodyB expressed in the local frame of BodyA. */
	double _fn[3];
	/** Total tangential force magnitude corrected to be consistent with
	friction constraints. */
	double _ftMag;
	/** Elastic tangential force NOT corrected to enforce friction
	constraints expressed in the local frame of BodyA. */
	double _ftp[3];
	/** Viscous tangential force NOT corrected to enforce friction
	constraints expressed in the local frame of BodyA. */
	double _ftv[3];
	/** Total tangential force applied to BodyB expressed in the local frame
	of BodyA.  Note that _ftA is not necessarily in the same direction as _tA
	because of the viscosity component. */
	double _ft[3];
	/** Correction in the spring force in order to enforce friction
	constraints.  It is the change made to the force applied to
   BodyA and is expressed in the local frame of BodyA. */
	double _dfFric[3];
	/** Pointer to a vector function that contains the velocity of _pA,
	expressed in the inertial frame, as a function of time. */
	VectorFunction *_vAFunction;
	/** Pointer to a vector function that contains the velocity of _pB,
	expressed in the inertial frame, as a function of time. */
	VectorFunction *_vBFunction;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ContactForce(std::string aBodyA="",std::string aBodyB="");
	ContactForce(const ContactForce &aForce);
	virtual ~ContactForce();
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	ContactForce& operator=(const ContactForce &aContact);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// NORMAL ON BODY A
	void setNormalA(const double aNormal[3]);
	void getNormalA(double rNormal[3]) const;
	// NORMAL ON BODY B
	void setNormalB(const double aNormal[3]);
	void getNormalB(double rNormal[3]) const;
	// NORMAL DISPLACEMENT VECTOR
	void getNormalDisplacement(double rDisplacement[3]) const;
	// NORMAL DISTANCE
	double getNormalDistance() const;
	// NORMAL VELOCITY
	void getNormalVelocity(double rVelocity[3]) const;
	// NORMAL SPEED
	double getNormalSpeed() const;
	// TANGENT ON BODY A
	void getTangent(double rTangent[3]) const;
	// TANGENTIAL DISPLACEMENT VECTOR
	void getTangentialDisplacement(double rDisplacement[3]) const;
	// TANGENTIAL DISTANCE
	double getTangentialDistance() const;
	// TANGENTIAL VELOCITY
	void getTangentialVelocity(double rVelocity[3]) const;
	// FORCE
	void getNormalForce(double rFP[3],double rFV[3],double rF[3]) const;
	void getTangentialForce(double rFP[3],double rFV[3],double rF[3]) const;
	// TANGENTIAL IMPEDANCE
	virtual double getInstantaneousTangentialStiffness() const;
	virtual double getInstantaneousTangentialViscosity() const;
	// NORMAL IMPEDANCE
	virtual double getInstantaneousNormalStiffness() const;
	virtual double getInstantaneousNormalViscosity() const;
	// FRICTION CORRECTION
	void getFrictionCorrection(double rDF[3]) const;
	// POINT A VELOCITY FUNCTION
	void setVelPointAFunction(VectorFunction* aVectorFunction);
	const VectorFunction* getVelPointAFunction() const;
	// POINT B VELOCITY FUNCTION
	void setVelPointBFunction(VectorFunction* aVectorFunction);
	const VectorFunction* getVelPointBFunction() const;



	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void updatePseudoStates();
	virtual void computeDisplacements();
	virtual void computeVelocities();

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	virtual bool check() const;

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	virtual void
		computeLineOfActionComponents(double rNormal[3],
		double rTangential[3]) const;

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode();

//=============================================================================
};	// END of class ContactForce

}; //namespace
//=============================================================================
//=============================================================================

#endif // __ContactForce_h__


