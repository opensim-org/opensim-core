// DerivCallbackSet.cpp
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
#include "DerivCallbackSet.h"
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
 * Note that the individual callbacks are not deleted by
 * this destructor.  To delete the callbacks, the caller must do so
 * individually, or the method Callback::deleteCallbacks() may be called.
 */
DerivCallbackSet::~DerivCallbackSet()
{
}
//_____________________________________________________________________________
/**
 * Construct an empty callback set for a model.
 */
DerivCallbackSet::DerivCallbackSet(Model *aModel) :
	CallbackSet(aModel)
{
	// NULL
	setNull();
}


//=============================================================================
// CONSTRUCTION AND DESTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void DerivCallbackSet::
setNull()
{
	// TYPE
	setType("DerivCallbackSet");
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the callback at an index.  This method uses the method
 * Array::get() and casts the returned void pointer as an
 * IntegCallback.
 *
 * @param aIndex Array index of the callback to be returned.
 * @return Callback at index aIndex.
 */
DerivCallback* DerivCallbackSet::
getDerivCallback(int aIndex) const
{
	return((DerivCallback*)get(aIndex));
}


//=============================================================================
// CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * This method is intended to be called from Model::deriv() after the
 * states of a model have been set (e.g., after Model::set() or
 * Model::setStates()).
 *
 * @param aT Current time in the simulation.
 * @param aX Current controls of the model.
 * @param aY Current states of the model
 */
void DerivCallbackSet::
set(double aT,double *aX,double *aY)
{
	int i;
	int size = getSize();
	DerivCallback *callback;
	for(i=0;i<size;i++) {
		callback = getDerivCallback(i);
		if(callback == NULL) continue;
		callback->set(aT,aX,aY);
	}
}
//_____________________________________________________________________________
/**
 * This method is intended to be called from Model::deriv() after 
 * actuation quantities have been computed (e.g., after
 * Model::computeActuation()).
 *
 * @param aT Current time in the simulation.
 * @param aX Current controls of the model.
 * @param aY Current states of the model
 */
void DerivCallbackSet::
computeActuation(double aT,double *aX,double *aY)
{
	int i;
	int size = getSize();
	DerivCallback *callback;
	for(i=0;i<size;i++) {
		callback = getDerivCallback(i);
		if(callback == NULL) continue;
		callback->computeActuation(aT,aX,aY);
	}
}
//_____________________________________________________________________________
/**
 * This method is intended to be called from Model::deriv() after 
 * actuator forces have been applied (e.g., after
 * Model::applyActuatorForces()).
 *
 * @param aT Current time in the simulation.
 * @param aX Current controls of the model.
 * @param aY Current states of the model
 */
void DerivCallbackSet::
applyActuation(double aT,double *aX,double *aY)
{
	int i;
	int size = getSize();
	DerivCallback *callback;
	for(i=0;i<size;i++) {
		callback = getDerivCallback(i);
		if(callback == NULL) continue;
		callback->applyActuation(aT,aX,aY);
	}
}
//_____________________________________________________________________________
/**
 * This method is intended to be called from Model::deriv() after 
 * contact quantities have been computed (e.g., after
 * Model::computeContact()).
 *
 * @param aT Current time in the simulation.
 * @param aX Current controls of the model.
 * @param aY Current states of the model
 */
void DerivCallbackSet::
computeContact(double aT,double *aX,double *aY)
{
	int i;
	int size = getSize();
	DerivCallback *callback;
	for(i=0;i<size;i++) {
		callback = getDerivCallback(i);
		if(callback == NULL) continue;
		callback->computeContact(aT,aX,aY);
	}
}
//_____________________________________________________________________________
/**
 * This method is intended to be called from Model::deriv() after 
 * contact forces have been applied (e.g., after
 * Model::applyContactForces()).
 *
 * @param aT Current time in the simulation.
 * @param aX Current controls of the model.
 * @param aY Current states of the model
 */
void DerivCallbackSet::
applyContact(double aT,double *aX,double *aY)
{
	int i;
	int size = getSize();
	DerivCallback *callback;
	for(i=0;i<size;i++) {
		callback = getDerivCallback(i);
		if(callback == NULL) continue;
		callback->applyContact(aT,aX,aY);
	}
}
//_____________________________________________________________________________
/**
 * This method is intended to be called from Model::deriv() after 
 * derivatives of the model states have been computed (e.g., after
 * Model::computeAccelerations(), ...).
 *
 * @param aT Current time in the simulation.
 * @param aX Current controls of the model.
 * @param aY Current states of the model
 */
void DerivCallbackSet::
computeDerivatives(double aT,double *aX,double *aY,double *aDY)
{
	int i;
	int size = getSize();
	DerivCallback *callback;
	for(i=0;i<size;i++) {
		callback = getDerivCallback(i);
		if(callback == NULL) continue;
		callback->computeDerivatives(aT,aX,aY,aDY);
	}
}
//_____________________________________________________________________________
/**
 * Calls reset() method of all deriv callbacks in this set.
 */
void DerivCallbackSet::
resetCallbacks()
{
	for(int i=0;i<getSize();i++) {
		DerivCallback *callback=getDerivCallback(i);
		if(callback) callback->reset();
	}
}

//_____________________________________________________________________________
/**
 * Call the step method for all deriv callbacks.  This method is called
 * after each successful integration time step and is intended to be used for
 * conducting analyses, driving animations, etc.
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
 */
void DerivCallbackSet::
step(double *aXPrev,double *aYPrev,double *aYPPrev,int aStep,double aDT,double aT,
	double *aX,double *aY,double *aYP,double *aDYDT,void *aClientData)
{
	int i;
	DerivCallback *callback;
	for(i=0;i<getSize();i++) {
		callback = getDerivCallback(i);
		if(callback == NULL) continue;
		callback->step(aXPrev,aYPrev,aYPPrev,aStep,aDT,aT,aX,aY,aYP,aDYDT,aClientData);
	}
}