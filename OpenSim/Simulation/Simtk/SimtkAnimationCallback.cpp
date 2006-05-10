// SimtkAnimationCallback.cpp
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
 * Author:  
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Simulation/Model/IntegCallbackSet.h>
#include "SimtkAnimationCallback.h"
#include <OpenSim/Tools/TransformChangeEvent.h>

//=============================================================================
// STATICS
//=============================================================================



using namespace OpenSim;
bool SimtkAnimationCallback::_busy = false;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimtkAnimationCallback::~SimtkAnimationCallback()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * Note that this constructor adds the callback to the model.  Derived
 * classes should not also add themselves to the model.
 *
 * @param aModel Model to which the callback mthods apply.
 */
SimtkAnimationCallback::SimtkAnimationCallback(Model *aModel) :
	IntegCallback(aModel)
{
	// NULL
	setNull();

	// We keep pointer to body's xform. Don't delete it on exit
	_transforms.setMemoryOwner(false);

	static double Orig[3] = { 0.0, 0.0, 0.0 };	// Zero 
	double t[3];	// Translation from sdfast
	double com[3];	// Center of mass
	double rot[3];	// Rotation angles 
	double dirCos[3][3];	// Direction cosines
	for(int i=0;i<_model->getNB();i++) {
		// get position from sdfast
		// adjust by com
		_model->getBody(i)->getCenterOfMass(com);
		for(int k=0; k < 3; k++)
			Orig[k] = -com[k];
		_model->getPosition(i, Orig, t);

		_model->getDirectionCosines(i,dirCos);
		_model->convertDirectionCosinesToAngles(dirCos,
			&rot[0],&rot[1],&rot[2]);
		// Initialize xform to identity
		_model->getBody(i)->getTransform().setOrientation(rot);
		_model->getBody(i)->getTransform().setPosition(t);
		_transforms.append(&_model->getBody(i)->getTransform());
	}

	_currentTime=0.0;
	// manually add callback to model
	_model->addIntegCallback(this);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for member variables.
 */
void SimtkAnimationCallback::
setNull()
{
	setType("SimtkAnimationCallback");
	setName("UNKNOWN");
	//_transforms.setSize(0);
	_currentTime=0.0;
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * This method is called from ui code to give user feedback about how far along 
 * is the simulation
 */
const double SimtkAnimationCallback::
getCurrentTime() const
{
	return _currentTime;
}

//=============================================================================
// CALLBACKS
//=============================================================================
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
 */
int SimtkAnimationCallback::
step(double *aXPrev,double *aYPrev,int aStep,double aDT,double aT,
	double *aX,double *aY,void *aClientData)
{
	if(_busy)
		return 0;

	getMutex();
	_currentTime = aT;
	// CHECK STEP INTERVAL
	if((aStep% getStepInterval())!=0) return 0;

	// MAYA TIME
	double realTime = aT * _model->getTimeNormConstant();
	printf("time = %lf\n",realTime);

	// SET STATES Why do we need this?
	//_model->set(aT,aX,aY);

	// LOOP OVER BODIES
	int i;
	static double Orig[3] = { 0.0, 0.0, 0.0 };	// Zero 
	double t[3];	// Translation from sdfast
	double com[3];	// Center of mass
	double rot[3];	// Rotation angles 
	double dirCos[3][3];	// Direction cosines
	for(i=0;i<_model->getNB();i++) {

		// TRANSLATION AND ROTATION
		Body *body = _model->getBody(i);
		body->getCenterOfMass(com);
		for(int k=0; k < 3; k++)
			Orig[k] = -com[k];
		_model->getPosition(i, Orig, t);

		_model->getDirectionCosines(i,dirCos);
		_model->convertDirectionCosinesToAngles(dirCos,
			&rot[0],&rot[1],&rot[2]);
		// Initialize xform to identity
		_transforms[i]->setOrientation(rot);
		_transforms[i]->setPosition(t);
		/* Create a transform change event and notify observers
		TransformChangeEvent	*xformChangeEvnt = new TransformChangeEvent(*body);
		body->setTransform(*_transforms[i]);
		body->notifyObservers(*xformChangeEvnt);
		delete xformChangeEvnt;*/
	}
	releaseMutex();
	return (0);
}
void SimtkAnimationCallback::getMutex()
{
	while(_busy)
		;
	_busy = true;
	return;
}

void SimtkAnimationCallback::releaseMutex()
{
	_busy = false;
}
