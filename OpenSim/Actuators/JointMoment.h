#ifndef _JointMoment_h_
#define _JointMoment_h_
// JointMoment.h
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

/*  
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "Actuators.h"
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Actuators/GeneralizedForceAtv.h>


//=============================================================================
//=============================================================================
/**
 * This class is the same as GeneralizedForceAtv except that it adds a
 * new property (optinal_negative_force) that allows one to specify a
 * different flexion strength and extension strength.
 *
 * Controls: excitation
 * States: activation
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class RDACTUATORS_API JointMoment : public GeneralizedForceAtv
{
//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Optimal negative force.  This optimal force parameter is used when
	the activation level is positive.  It is the mechanism used for having
	a different flexion and extension strenthgs */
	PropertyDbl _propOptimalNegForce;

	// REFERENCES
	double &_optimalNegForce;

//=============================================================================
// METHODS
//=============================================================================
public:
	JointMoment(std::string aQName="");
	JointMoment(DOMElement *aElement);
	JointMoment(const JointMoment &aActuator);
	virtual ~JointMoment();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
private:
	void setNull();
	void setupProperties();
	void copyData(const JointMoment &aActuator);

public:

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
	JointMoment&
		operator=(const JointMoment &aActuator);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// OPTIMAL NEGATIVE FORCE
	void setOptimalNegativeForce(double aOptNegForce);
	double getOptimalNegativeForce() const;

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeActuation();

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode();

//=============================================================================
};	// END of class JointMoment

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __JointMoment_h__
