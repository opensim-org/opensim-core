#ifndef _GeneralizedForce_h_
#define _GeneralizedForce_h_
// GeneralizedForce.h
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


#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyInt.h>
#include <OpenSim/Simulation/SIMM/AbstractActuator.h>
#include <OpenSim/Simulation/SIMM/AbstractSpeed.h>


//=============================================================================
//=============================================================================
/**
 * A class that supports the application of a generalized force to a model.
 * This actuator has no states; the control is simply the force to
 * be applied to the model.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class AbstractCoordinate;

class RDSIMULATION_API GeneralizedForce : public AbstractActuator
{
//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Name of coordinate to which the generalized force is applied. */
	PropertyStr _propQName;
	/** Optimal force. */
	PropertyDbl _propOptimalForce;

	// REFERENCES
	std::string& _qName;
	double &_optimalForce;

	/** Coordinate to which the generalized force is applied. */
	AbstractCoordinate *_q;
	AbstractSpeed *_u;

	/** Temporary work array for holding generalized speeds. */
	double *_utmp;

	/** Excitation (control 0). */
	double _excitation;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	GeneralizedForce(std::string aQName="");
	GeneralizedForce(DOMElement *aElement);
	GeneralizedForce(const GeneralizedForce &aGenForce);
	virtual ~GeneralizedForce();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
	void copyData(const GeneralizedForce &aGenForce);
private:
	void setNull();
	void setupProperties();
	

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	GeneralizedForce& operator=(const GeneralizedForce &aGenForce);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// GENERALIZED COORDINATE
	void setQ(AbstractCoordinate* aQ);
	AbstractCoordinate* getQ() const;
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

	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	virtual bool check() const;
	virtual bool isQValid() const;
	// Setup method to initialize Body references
	void setup(AbstractModel* aModel);

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode();

//=============================================================================
};	// END of class GeneralizedForce

}; //namespace
//=============================================================================
//=============================================================================

#endif // __GeneralizedForce_h__


