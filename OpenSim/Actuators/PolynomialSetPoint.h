#ifndef _PolynomialSetPoint_h_
#define _PolynomialSetPoint_h_
// PolynomialSetPoint.h
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
#include "SetPoint.h"


//=============================================================================
//=============================================================================
/**
 * Contact modeled with a polynomal spring in the normal direction and
 * a moving set point in the tangential plane.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class RDACTUATORS_API PolynomialSetPoint : public SetPoint
{

//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Normal stiffness constant.  This constant is not to be confused with
	the instantaneous stiffness. */
	PropertyDbl _propKNP;
	/** Normal viscosity constant.  This constant is not to be confused with
	the instantaneous viscosity. */
	PropertyDbl _propKNV;
	/** Stiffness power- power to which normal spring displacement is raised.
	The power is 1.0 by default. */
	PropertyDbl _propPowerNP;
	/** Viscosity power- power to which normal spring velocity is raised.
	The power is 1.0 by default. */
	PropertyDbl _propPowerNV;

	// REFERENCES
	double &_kNP;
	double &_kNV;
	double &_powNP;
	double &_powNV;


	/** Instantaneous stiffness in the normal direction. */
	double _knp;
	/** Instantaneous viscosity in the normal direction. */
	double _knv;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PolynomialSetPoint(int aBodyA=0,int aBodyB=0);
	PolynomialSetPoint(DOMElement *aElement);
	PolynomialSetPoint(const PolynomialSetPoint &aContact);
	virtual ~PolynomialSetPoint();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	PolynomialSetPoint& operator=(const PolynomialSetPoint &aActuator);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// NORMAL IMPEDANCE
	virtual double getInstantaneousNormalStiffness() const;
	virtual double getInstantaneousNormalViscosity() const;
	// NORMAL STIFFNESS
	void setNormalStiffnessConstant(double aKNP);
	double getNormalStiffnessConstant() const;
	// NORMAL VISCOSITY
	void setNormalViscosityConstant(double aKNV);
	double getNormalViscosityConstant() const;
	// STIFFNESS POWER
	void setStiffnessPower(double aPower);
	double getStiffnessPower() const;
	// VISCOSITY POWER
	void setViscosityPower(double aPower);
	double getViscosityPower() const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeActuation();

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------


	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	virtual bool check() const;

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode();

//=============================================================================
};	// END of class PolynomialSetPoint

}; //namespace
//=============================================================================
//=============================================================================

#endif // __PolynomialSetPoint_h__


