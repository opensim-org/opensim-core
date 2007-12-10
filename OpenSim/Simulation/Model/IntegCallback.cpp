// IntegCallback.cpp
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


//=============================================================================
// INCLUDES
//=============================================================================
#include "IntegCallbackSet.h"
#include "IntegCallback.h"
#include <OpenSim/Common/PropertyInt.h>
#include "Model.h"
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
IntegCallback::IntegCallback(Model *aModel) :
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
IntegCallback::IntegCallback(const std::string &aFileName, bool aUpdateFromXMLNode):
Callback(aFileName, false),
	_stepInterval(_stepIntervalProp.getValueInt())
{
	setNull();
	if(aUpdateFromXMLNode) updateFromXMLNode();
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
	_stepIntervalProp.setComment("Specifies how often to store results during a simulation. "
		"More specifically, the interval (a positive integer) specifies how many successful "
		"integration steps should be taken before results are recorded again.");
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
// INTEGCALLBACK
//=============================================================================
//_____________________________________________________________________________
/**
 * Return whether or not to proceed with this callback.
 * The callback will not proceed (i.e., returns false) if either the
 * analysis is turned off or if aStep is not an even multiple of
 * the step interval set @see rdStepCallback.
 *
 * @return True or False.
 */
bool IntegCallback::
proceed(int aStep)
{
	return(getOn() && ((aStep%_stepInterval)==0));
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
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int IntegCallback::
begin(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
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
 * @param aYPPrev Pseudo state values at the previous time step.
 * @param aStep Number of integrations steps that have been completed.
 * @param aDT Size of the time step that WAS just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int IntegCallback::
step(double *aXPrev,double *aYPrev,double *aYPPrev,int aStep,double aDT,double aT,
	double *aX,double *aY,double *aYP,double *aDYDT,void *aClientData)
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
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int IntegCallback::
end(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
	 void *aClientData)
{
	//printf("IntegCallback.end: %s.\n",getName());
	return(0);
}

