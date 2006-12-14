// IntegCallback.cpp
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


//=============================================================================
// INCLUDES
//=============================================================================
#include "IntegCallbackSet.h"
#include "IntegCallback.h"
#include <OpenSim/Tools/PropertyInt.h>
#include <OpenSim/Simulation/Simm/AbstractModel.h>
//=============================================================================
// STATICS
//=============================================================================

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
IntegCallback::~IntegCallback()
{
}

//_____________________________________________________________________________
/**
 * Constructor.
 *
 * Note that this constructor adds the callback to the model.  Derived
 * classes should not also add themselves to the model.
 *
 * @param aModel Model to which the callback mthods apply.
 */
IntegCallback::IntegCallback(AbstractModel *aModel) :
	Callback(aModel),
	_stepInterval(_stepIntervalProp.getValueInt())
{
	// NULL
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy Constructor.
 *
 */
IntegCallback::IntegCallback(const IntegCallback &aIntegCallback):
Callback(aIntegCallback),
	_stepInterval(_stepIntervalProp.getValueInt())
{
	setNull();
	*this = aIntegCallback;
}
//_____________________________________________________________________________
/**
 * Constructor from an xml file
 *
 */
IntegCallback::IntegCallback(const std::string &aFileName):
Callback(aFileName),
	_stepInterval(_stepIntervalProp.getValueInt())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Constructor from an xml element
 *
 */
IntegCallback::IntegCallback(DOMElement *aElement):
	Callback(aElement),
	_stepInterval(_stepIntervalProp.getValueInt())
{
	setNull();

	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Constructor from a clone
 *
 */
Object* IntegCallback::copy() const
{
	return(new IntegCallback(*this));
}
//_____________________________________________________________________________
/**
 * Virtual Constructor from an xml element
 *
 */
Object* IntegCallback::copy(DOMElement *aElement) const
{
	IntegCallback *c = new IntegCallback(aElement);
	*c = *this;
	c->updateFromXMLNode();
	return(c);
} 

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for member variables.
 */
void IntegCallback::
setNull()
{
	setupProperties();

	setType("IntegCallback");
	_stepInterval = 1;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void IntegCallback::
setupProperties()
{
	_stepIntervalProp.setName("step_interval");
	_propertySet.append( &_stepIntervalProp );
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
IntegCallback& IntegCallback::
operator=(const IntegCallback &aObject)
{
	// BASE CLASS
	Callback::operator=(aObject);

	// Class Members
	setStepInterval(aObject.getStepInterval());

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the step interval.
 *
 * The step interval is used to specify how many integration steps must
 * go by before the IntegCallback::step() method is executed.
 * Specifically, unless the step number divided by the step interval
 * has no remainder (i.e., (step % stepInterval) == 0), the step
 * method is not executed.
 *
 * @param aStepInterval Step interval. Should be 1 or greater.
 */
void IntegCallback::
setStepInterval(int aStepInterval)
{
	_stepInterval = aStepInterval;
	if(_stepInterval<1) _stepInterval= 1;
}
//_____________________________________________________________________________
/**
 * Get the step interval.
 *
 * The step interval is used to specify how many integration steps must
 * go by before the IntegCallback::step() method is executed.
 * Specifically, unless the step number divided by the step interval
 * has no remainder (i.e., (step % stepInterval) == 0), the step
 * method is not executed.
 *
 * @return Step interval.
 */
int IntegCallback::
getStepInterval() const
{
	return(_stepInterval);
}


//=============================================================================
// CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an integration and is intended
 * to be used for any initializations that are necessary.
 *
 * Override this method in derived classes.
 *
 * @param aStep Number of integrations steps that have been completed.
 * @param aDT Size of the integration time step that will be attempted.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int IntegCallback::
begin(int aStep,double aDT,double aT,double *aX,double *aY,
	void *aClientData)
{
	//printf("IntegCallback.begin: %s.\n",getName());
	return (0);
}
//_____________________________________________________________________________
/**
 * This method is called after each successful integration time step and is
 * intended to be used for conducting analyses, driving animations, etc.
 *
 * Override this method in derived classes.
 *
 * @param aXPrev Control values at the previous time step.
 * @param aYPrev State values at the previous time step.
 * @param aStep Number of integrations steps that have been completed.
 * @param aDT Size of the time step that WAS just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int IntegCallback::
step(double *aXPrev,double *aYPrev,int aStep,double aDT,double aT,
	double *aX,double *aY,void *aClientData)
{
	//printf("IntegCallback.step: %s.\n",getName());
	return (0);
}
//_____________________________________________________________________________
/**
 * This method is called after an integration has been completed and is
 * intended to be used for performing any finalizations necessary.
 *
 * Override this method in derived classes.
 *
 * @param aStep Number of integrations steps that have been completed.
 * @param aDT Size of the time step that WAS just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int IntegCallback::
end(int aStep,double aDT,double aT,double *aX,double *aY,
	 void *aClientData)
{
	//printf("IntegCallback.end: %s.\n",getName());
	return(0);
}

