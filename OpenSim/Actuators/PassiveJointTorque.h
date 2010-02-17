#ifndef _PassiveJointTorque_h_
#define _PassiveJointTorque_h_
// PassiveJointTorque.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Kate Holzbaur
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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


//=============================================================================
// INCLUDES
//=============================================================================
#include "osimActuatorsDLL.h"
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Actuators/CoordinateActuator.h>


//=============================================================================
//=============================================================================
/**
 * An actuator that exerts passive elastic and damping forces at a
 * generalized coordinate.
 *
 * @author Frank C. Anderson, Kate Holzbaur
 * @version 1.0
 */
namespace OpenSim { 

class OSIMACTUATORS_API PassiveJointTorque : public CoordinateActuator 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Passive torque parameters. */
	PropertyDbl _propSlope1;
	double &_slope1;
	
	PropertyDbl _propLimit1;
	double &_limit1;
	
	PropertyDbl _propSlope2;
	double &_slope2;
	
	PropertyDbl _propLimit2;
	double &_limit2;

	PropertyDbl _propOffset;
	double &_offset;
	
	/** Damping parameters. */
	PropertyDbl _propDamping;
	double &_damping;

//=============================================================================
// METHODS
//=============================================================================
public:
	PassiveJointTorque(std::string aQName="");
	PassiveJointTorque(const PassiveJointTorque &aActuator);
	virtual ~PassiveJointTorque();
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();
	void copyData(const PassiveJointTorque &aActuator);

public:

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
	PassiveJointTorque&
		operator=(const PassiveJointTorque &aActuator);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// Parameters
	void setSlope1(double aSlope1);
	double getSlope1() const;
	
	void setLimit1(double aLimit1);
	double getLimit1() const;
	
	void setSlope2(double aSlope2);
	double getSlope2() const;
	
	void setLimit2(double aLimit2);
	double getLimit2() const;
	
	void setOffset(double aOffset);
	double getOffset() const;
	
	void setDamping(double aDamping);
	double getDamping() const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double computeActuation( const SimTK::State& s );

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	//virtual void updateFromXMLNode();

	OPENSIM_DECLARE_DERIVED(PassiveJointTorque,Actuator);

//=============================================================================
};	// END of class PassiveJointTorque

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __PassiveJointTorque_h__
