#ifndef _SetPoint_h_
#define _SetPoint_h_
// SetPoint.h
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

#include "osimActuatorsDLL.h"
#include <OpenSim/Common/PropertyDbl.h>
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

class OSIMACTUATORS_API SetPoint : public ContactForce
{
//=============================================================================
// DATA
//=============================================================================
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
	SetPoint(std::string aBodyA="",std::string aBodyB="");
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
		computeTangentialForce(double aNormalForce,SimTK::Vec3& rTangentForce,
		SimTK::Vec3& rDFFric);
	bool
		computeNewSetPoint(double aTangentForce,SimTK::Vec3& rSetPoint) const;

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode();

	OPENSIM_DECLARE_DERIVED(SetPoint, AbstractActuator);

//=============================================================================
};	// END of class SetPoint

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SetPoint_h__


