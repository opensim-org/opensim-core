#ifndef _PolynomialSetPoint_h_
#define _PolynomialSetPoint_h_
// PolynomialSetPoint.h
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

#include <OpenSim/Common/PropertyDbl.h>
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

class OSIMACTUATORS_API PolynomialSetPoint : public SetPoint
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
	PolynomialSetPoint(std::string aBodyA="",std::string aBodyB="");
	PolynomialSetPoint(const PolynomialSetPoint &aContact);
	virtual ~PolynomialSetPoint();
	virtual Object* copy() const;
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

	OPENSIM_DECLARE_DERIVED(PolynomialSetPoint, AbstractActuator);

//=============================================================================
};	// END of class PolynomialSetPoint

}; //namespace
//=============================================================================
//=============================================================================

#endif // __PolynomialSetPoint_h__


