// DerivCallbackSet.cpp
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
#include "DerivCallbackSet.h"


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
