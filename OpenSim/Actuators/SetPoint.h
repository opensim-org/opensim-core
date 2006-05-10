#ifndef _SetPoint_h_
#define _SetPoint_h_
// SetPoint.h
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

#include "Actuators.h"
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ContactForce.h>


//=============================================================================
//=============================================================================
/**
 * A class that supports a contact force with a moving set point or spring
 * zero.  The set point moves on the body whose direction and surface
 * normal are active (BodyA).
 *
 * A set point has three pseudostates: the coordinates of contact PointA. 
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class RDACTUATORS_API SetPoint : public ContactForce
{
//=============================================================================
// DATA
//=============================================================================
private:
	static const std::string YP0_NAME;
	static const std::string YP1_NAME;
	static const std::string YP2_NAME;
protected:
	// PROPERTIES
	/** Constant stiffness in the tangential direction. */
	PropertyDbl _propKTP;
	/** Constant viscosity in the tangential direction. */
	PropertyDbl _propKTV;
	/** Coefficient of friction. */
	PropertyDbl _propMU;

	// REFERENCES
	double &_ktp;
	double &_ktv;
	double &_mu;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SetPoint(int aBodyA=0,int aBodyB=0,int aNYP=3);
	SetPoint(DOMElement *aElement,int aNYP=3);
	SetPoint(const SetPoint &aContact);
	virtual ~SetPoint();
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	SetPoint& operator=(const SetPoint &aSetPoint);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// PSEUDOSTATES
	virtual int getNYP() const;
	virtual const std::string& getPseudoStateName(int aIndex) const;
	virtual int getPseudoStateIndex(const std::string &aName) const;
	virtual void setPseudoStates(const double aX[]);
	virtual void setPseudoState(int aIndex,double aValue);
	virtual void setPseudoState(const std::string &aName,double aValue);
	virtual void getPseudoStates(double rX[]) const;
	virtual double getPseudoState(int aIndex) const;
	virtual double getPseudoState(const std::string &aName) const;
	// TANGENTIAL IMPEDANCE
	virtual double getInstantaneousTangentialStiffness() const;
	virtual double getInstantaneousTangentialViscosity() const;
	// TANGENTIAL STIFFNESS
	void setTangentialStiffness(double aKTP);
	double getTangentialStiffness() const;
	// TANGENTIAL VISCOSITY
	void setTangentialViscosity(double aKTV);
	double getTangentialViscosity() const;
	// COEFFICIENT OF FRICTION
	void setFriction(double aMU);
	double getFriction() const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void updatePseudoStates();

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
	double
		computeTangentialForce(double aNormalForce,double rTangentForce[3],
		double rDFFric[3]);
	bool
		computeNewSetPoint(double aTangentForce,double rSetPoint[3]) const;

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode();

//=============================================================================
};	// END of class SetPoint

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SetPoint_h__


