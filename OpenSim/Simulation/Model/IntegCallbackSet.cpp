// IntegCallbackSet.cpp
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
IntegCallbackSet::~IntegCallbackSet()
{
}
//_____________________________________________________________________________
/**
 * Construct an empty callback set for a model.
 */
IntegCallbackSet::IntegCallbackSet(Model *aModel)
{
	setType("IntegCallbackSet");
	setNull();

	_model = aModel;
}


//=============================================================================
// CONSTRUCTION AND DESTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void IntegCallbackSet::
setNull()
{
	_model = NULL;
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get a pointer to the model which is actuated.
 *
 * @return Pointer to the model.
 */
Model* IntegCallbackSet::
getModel()
{
	return(_model);
}

//-----------------------------------------------------------------------------
// ON & OFF
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set all the callbacks either on or off.
 *
 * @param aTrueFalse Arguement that, if true, results in all callbacks
 * being turned on; if false, all callbacks are turned off.
 */
void IntegCallbackSet::
setOn(bool aTrueFalse)
{
	for(int i=0;i<getSize();i++) if(get(i)) get(i)->setOn(aTrueFalse);
}

void IntegCallbackSet::
setOn(const Array<bool> &aOn) 
{
	if(aOn.getSize()!=getSize()) throw Exception("IntegCallbackSet.setOn: ERROR- incompatible array sizes",__FILE__,__LINE__);
	for(int i=0; i<getSize(); i++) if(get(i)) get(i)->setOn(aOn[i]);
}

Array<bool> IntegCallbackSet::
getOn() const
{
	Array<bool> on(false,getSize());
	for(int i=0; i<getSize(); i++) if(get(i)) on[i] = get(i)->getOn();
	return on;
}

//-----------------------------------------------------------------------------
// INTEGRATION CALLBACK
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the callback at an index.  This method uses the method
 * Array::get() and casts the returned void pointer as an
 * IntegCallback.
 *
 * @param aIndex Array index of the callback to be returned.
 * @return Callback at index aIndex.
 */
IntegCallback* IntegCallbackSet::
getIntegCallback(int aIndex) const
{
	return((IntegCallback*)get(aIndex));
}


//=============================================================================
// CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * Call the begin method for all integration callbacks.  This method is
 * called at the beginning of an integration and is intended to be used for
 * any initializations that are necessary.
 *
 * @param aStep Number of integrations steps that have been completed.
 * @param aDT Size of the integration time step that will be attempted.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives
 * @param aClientData General use pointer for sending in client data.
 */
void IntegCallbackSet::
begin(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,void *aClientData)
{
	int i;
	IntegCallback *callback;
	for(i=0;i<getSize();i++) {
		callback = getIntegCallback(i);
		if(callback == NULL) continue;
		callback->begin(aStep,aDT,aT,aX,aY,aYP,aDYDT,aClientData);
	}
}
//_____________________________________________________________________________
/**
 * Call the step method for all integration callbacks.  This method is called
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
void IntegCallbackSet::
step(double *aXPrev,double *aYPrev,double *aYPPrev,int aStep,double aDT,double aT,
	double *aX,double *aY,double *aYP,double *aDYDT,void *aClientData)
{
	int i;
	IntegCallback *callback;
	for(i=0;i<getSize();i++) {
		callback = getIntegCallback(i);
		if(callback == NULL) continue;
		callback->step(aXPrev,aYPrev,aYPPrev,aStep,aDT,aT,aX,aY,aYP,aDYDT,aClientData);
	}
}
//_____________________________________________________________________________
/**
 * Call the end method for all integration callbacks.  This method is called
 * after an integration has been completed and is intended to be used for
 * performing any finalizations necessary.
 *
 * @param aStep Number of integrations steps that have been completed.
 * @param aDT Size of the time step that WAS just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 */
void IntegCallbackSet::
end(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,void *aClientData)
{
	int i;
	IntegCallback *callback;
	for(i=0;i<getSize();i++) {
		callback = getIntegCallback(i);
		if(callback == NULL) continue;
		callback->end(aStep,aDT,aT,aX,aY,aYP,aDYDT,aClientData);
	}
}


