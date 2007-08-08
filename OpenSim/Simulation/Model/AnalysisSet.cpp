// AnalysisSet.cpp
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
#include "AnalysisSet.h"
#include "Model.h"


using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 * Note that the individual callbacks are not deleted by
 * this destructor.  To delete the callbacks, the caller must do so
 * individually, or the method Callback::deleteCallbacks() may be called.
 */
AnalysisSet::~AnalysisSet()
{
}
//_____________________________________________________________________________
/**
 * Construct an empty analysis set.
 *
 * @param aModel Model for the analysis set.
 */
AnalysisSet::AnalysisSet(Model *aModel)
{
	setType("AnalysisSet");
	setNull();
}
//_____________________________________________________________________________
/**
 * Construct an analysis set from file.
 *
 * @param aFileName Name of the file.
 */
AnalysisSet::AnalysisSet(const string &aFileName) :
	Set<Analysis>(aFileName, false)
{
	setType("AnalysisSet");
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSet Analysis set to be copied.
 */
AnalysisSet::AnalysisSet(const AnalysisSet &aSet) :
	Set<Analysis>(aSet)
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy this analysis set.
 *
 * @return Copy of this analysis set.
 */
Object* AnalysisSet::
copy() const
{
	AnalysisSet *set = new AnalysisSet(*this);
	return(set);
}


//=============================================================================
// CONSTRUCTION AND DESTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void AnalysisSet::
setNull()
{
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the model for all analyses in the set.
 *
 * @param aModel Pointer to the model.
 */
void AnalysisSet::
setModel(Model *aModel)
{
	int i;
	int size = getSize();
	Analysis *analysis;
	for(i=0;i<size;i++) {
		analysis = get(i);
		if(analysis==NULL) continue;
		analysis->setModel(aModel);
	}
}
//_____________________________________________________________________________
/**
 * Get a pointer to the model which is actuated.
 *
 * @return Pointer to the model.
 */
Model* AnalysisSet::
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
void AnalysisSet::
setOn(bool aTrueFalse)
{
	int i;
	Callback *callback;
	for(i=0;i<getSize();i++) {
		callback = get(i);
		if(callback==NULL) continue;
		callback->setOn(aTrueFalse);
	}
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
void AnalysisSet::
begin(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,void *aClientData)
{
	int i;
	IntegCallback *callback;
	for(i=0;i<getSize();i++) {
		callback = get(i);
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
 * @param aDYDT Current state derivatives
 * @param aClientData General use pointer for sending in client data.
 */
void AnalysisSet::
step(double *aXPrev,double *aYPrev,double *aYPPrev,int aStep,double aDT,double aT,
	double *aX,double *aY,double *aYP,double *aDYDT,void *aClientData)
{
	int i;
	IntegCallback *callback;
	for(i=0;i<getSize();i++) {
		callback = get(i);
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
 * @param aDYDT Current state derivatives
 * @param aClientData General use pointer for sending in client data.
 */
void AnalysisSet::
end(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,void *aClientData)
{
	int i;
	IntegCallback *callback;
	for(i=0;i<getSize();i++) {
		callback = get(i);
		if(callback == NULL) continue;
		callback->end(aStep,aDT,aT,aX,aY,aYP,aDYDT,aClientData);
	}
}



//=============================================================================
// RESULTS
//=============================================================================
//_____________________________________________________________________________
/**
 * Print the results of all analyses in the set.
 *
 * @param aIndex Array index of the callback to be returned.
 * @return Callback at index aIndex.
 */
void AnalysisSet::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	int i;
	int size = getSize();
	Analysis *analysis;
	for(i=0;i<size;i++) {
		analysis = get(i);
		if(analysis==NULL) continue;
		if(analysis->getPrintResultFiles()) analysis->printResults(aBaseName,aDir,aDT,aExtension);
	}
}
//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Fills the passed in analysis set with all available (i.e. registered) analyses.
 */
void AnalysisSet::
getAvailableAnalyses(AnalysisSet& as)
{
	Object::template getRegisteredObjectsOfGivenType<Analysis>(as._objects);
}
