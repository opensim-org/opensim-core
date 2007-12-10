#ifndef _LinearSetPoint_h_
#define _LinearSetPoint_h_
// LinearSetPoint.h
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

#include <string>
#include <OpenSim/Common/PropertyDbl.h>
#include "SetPoint.h"


//=============================================================================
//=============================================================================
/**
 * Contact modeled with a linear spring in the vertical direction and
 * a moving set point to model friction.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim {

class AbstractBody;

class OSIMACTUATORS_API LinearSetPoint : public SetPoint
{
//=============================================================================
// MEMBERS
//=============================================================================
	// PROPERTIES
	/** Constant stiffness in the normal direction. */
	PropertyDbl _propKNP;
	/** Constant viscosity in the normal direction. */
	PropertyDbl _propKNV;

	// REFERENCES
	double &_knp;
	double &_knv;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	LinearSetPoint(std::string aBodyA="",std::string aBodyB="");
	LinearSetPoint(const LinearSetPoint &aContact);
	virtual ~LinearSetPoint();
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	LinearSetPoint& operator=(const LinearSetPoint &aActuator);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// NORMAL IMPEDANCE
	virtual double getInstantaneousNormalStiffness() const;
	virtual double getInstantaneousNormalViscosity() const;
	// NORMAL STIFFNESS
	void setNormalStiffness(double aKNP);
	double getNormalStiffness() const;
	// NORMAL VISCOSITY
	void setNormalViscosity(double aKNV);
	double getNormalViscosity() const;

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

	OPENSIM_DECLARE_DERIVED(LinearSetPoint, AbstractActuator);

//=============================================================================
};	// END of class LinearSetPoint

}; //namespace
//=============================================================================
//=============================================================================

#endif // __LinearSetPoint_h__


