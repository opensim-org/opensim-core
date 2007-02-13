#ifndef _Torque_h_
#define _Torque_h_
// Torque.h
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

#include <string>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyInt.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/Function.h>
#include <OpenSim/Tools/VectorFunction.h>
#include <OpenSim/Simulation/SIMM/AbstractActuator.h>


//=============================================================================
//=============================================================================
/**
 * A class that supports the application of a torque between two bodies, BodyA
 * and BodyB.  This actuator has no states; the control is simply the
 * excitation of the actuator.  The torque is the optimal force multiplied
 * by the excitation.  So the excitation is a simply normalized torque
 * (i.e., torque / optimal force).  The direction of the torque is
 * expressed in the body-local frame of BodyA.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class AbstractBody;

class RDSIMULATION_API Torque : public AbstractActuator
{

//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** name of BodyA. */
	PropertyStr _propBodyAName;
	/** Unit vector expressed in the local frame of BodyA that
	specifies the direction a positive torque is applied to BodyA.
	(serialized) */
	PropertyDblArray _propUnitVectorA;
	/** name of BodyB. */
	PropertyStr _propBodyBName;
	/** Optimal force. */
	PropertyDbl _propOptimalForce;

	// REFERENCES
	std::string& _bodyAName;
	Array<double> &_uA;
	std::string& _bodyBName;
	double &_optimalForce;

	AbstractBody *_bA;
	AbstractBody *_bB;

	/** Unit vector expressed in the local frame of BodyB that
	specifies the direction a positive actuator force is applied to BodyB. */
	double _uB[3];

	/** Excitation (control 0). */
	double _excitation;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Torque(const std::string &aBodyAName="",const std::string &aBodyBName="");
	Torque(const Torque &aTorque);
	virtual ~Torque();
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	Torque& operator=(const Torque &aTorque);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// BODY A
	void setBodyA(AbstractBody* aBody);
	AbstractBody* getBodyA() const;
	// DIRECTION A
	void setDirectionA(const double aDirection[3]);
	void getDirectionA(double rDirection[3]) const;
	// BODY B
	void setBodyB(AbstractBody* aBody);
	AbstractBody* getBodyB() const;
	// DIRECTION B
	void getDirectionB(double rDirection[3]) const;
	// OPTIMAL FORCE
	void setOptimalForce(double aOptimalForce);
	double getOptimalForce() const;
	// STRESS
	double getStress() const;

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------
	virtual void apply();

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeActuation();
	void computeDirectionForBodyB();
	void computeSpeed();

	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	virtual bool check() const;
	// Setup method to initialize Body references
	void setup(AbstractModel* aModel);

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------

//=============================================================================
};	// END of class Torque

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Torque_h__


