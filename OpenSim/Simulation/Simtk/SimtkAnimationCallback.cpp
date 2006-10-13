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
#include <OpenSim/Simulation/SIMM/SimmBody.h>
#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include "SimtkAnimationCallback.h"

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

	// We keep pointer to body's xform. Don't delete them on exit
	_transforms = new Transform[_model->getNB()];

	static double Orig[3] = { 0.0, 0.0, 0.0 };	// Zero 
	std::string modelName = _model->getName();
	SimmModel *simmModel = dynamic_cast<SimmModel*>(_model);
	if (simmModel){
		SimmBody* gnd = simmModel->getSimmKinematicsEngine().getGroundBodyPtr();
		SimmBodySet& bodySet = simmModel->getSimmKinematicsEngine().getBodies();
		double dirCos[3][3];
		for(int i=0;i<_model->getNB();i++) {
			double stdFrame[3][3] = {{ 1.0, 0.0, 0.0 },
									{0., 1., 0.},
									{0., 0., 1.}};	 
			double Orig[3] = { 0.0, 0.0, 0.0 };
			SimmBody *body = bodySet.get(i);
			simmModel->getSimmKinematicsEngine().convertVector(stdFrame[0], body, gnd);
            simmModel->getSimmKinematicsEngine().convertVector(stdFrame[1], body, gnd);
            simmModel->getSimmKinematicsEngine().convertVector(stdFrame[2], body, gnd);
            simmModel->getSimmKinematicsEngine().convertPoint(Orig, body, gnd);
			for(int j=0; j <3; j++)
				for(int k=0; k <3; k++)
					dirCos[j][k]=stdFrame[j][k];
			_transforms[i].setRotationSubmatrix(dirCos);
			_transforms[i].setPosition(Orig);
		}
	}
	else {	// SDFast?
	}
	_currentTime=0.0;
	
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

	getMutex();	// Intention is to make sure xforms that are cached are consistent from the same time

	_currentTime = aT;
	// CHECK STEP INTERVAL
	if((aStep% getStepInterval())!=0) return 0;

	// OpenSim display time
	double realTime = aT * _model->getTimeNormConstant();
	printf("Callback time = %lf\n",realTime);

	// LOOP OVER BODIES
	double t[3];	// Translation from sdfast
	double dirCos[3][3];	// Direction cosines
	SimmModel *simmModel = dynamic_cast<SimmModel*>(_model);
	if (simmModel){
		SimmBody* gnd = simmModel->getSimmKinematicsEngine().getGroundBodyPtr();
		SimmBodySet& bodySet = simmModel->getSimmKinematicsEngine().getBodies();
		for(int i=0;i<_model->getNB();i++) {
			double stdFrame[3][3] = {{ 1.0, 0.0, 0.0 },
									{0., 1., 0.},
									{0., 0., 1.}};	 
			double Orig[3] = { 0.0, 0.0, 0.0 };
			SimmBody *body = bodySet.get(i);
			simmModel->getSimmKinematicsEngine().convertVector(stdFrame[0], body, gnd);
            simmModel->getSimmKinematicsEngine().convertVector(stdFrame[1], body, gnd);
            simmModel->getSimmKinematicsEngine().convertVector(stdFrame[2], body, gnd);
            simmModel->getSimmKinematicsEngine().convertPoint(Orig, body, gnd);
			// Assign dirCos
			for(int j=0; j <3; j++)
				for(int k=0; k <3; k++)
					dirCos[j][k]=stdFrame[j][k];

			_transforms[i].setRotationSubmatrix(dirCos);
			_transforms[i].setPosition(Orig);
		}
	}
	else {	// SDFast?
		double com[3];	// Center of mass
		double Orig[3] = { 0.0, 0.0, 0.0 };
		for(int i=0;i<_model->getNB();i++) {
			// get position from sdfast
			// adjust by com
			_model->getBody(i)->getCenterOfMass(com);
			for(int k=0; k < 3; k++)
				Orig[k] = -_offsets[3*i+k];
			_model->getPosition(i, Orig, t);

			_model->getDirectionCosines(i,dirCos);
			// Initialize xform to identity
			_transforms[i].setPosition(t);
			_transforms[i].setRotationSubmatrix(dirCos);

		}
	}
	releaseMutex();
	// Create a transform change event and notify observers
	/*TransformChangeEvent	*xformChangeEvnt = new TransformChangeEvent(*body);
	body->notifyObservers(*xformChangeEvnt);
	delete xformChangeEvnt;*/

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

const Transform* SimtkAnimationCallback::getBodyTransform(int index) const
{
	return &_transforms[index];
}
/**
 * Cache Coms for bodies so that we can get the xform for the bodies from SDFast in one go with 
 * minimal computation on our side. If displaying a SimmModel for example in IK then offsets are set to 0
 */
void SimtkAnimationCallback::extractOffsets(SimmModel& displayModel)
{
	_offsets = new double[displayModel.getNB()*3];
	int i;
	SimmModel *simmModel = dynamic_cast<SimmModel*>(_model);
	if (simmModel){	// Just set offsets to zero we're displaying a SimmModel and Running SimmModel too
		for(i=0;i<_model->getNB();i++){
			_offsets[i*3]=_offsets[i*3+1]=_offsets[i*3+2]=0.0;
		}
		return;
	}
	double bodyCom[3];

	
	for(i=0;i<_model->getNB();i++) {
	// TRANSLATION AND ROTATION
		Body *body = _model->getBody(i);
		std::string bodyName = body->getName();
		SimmBody* sBody = displayModel.getBodies().get(bodyName);
		if (sBody != 0){
			sBody->getMassCenter(bodyCom);
			for(int j=0; j<3; j++){
				_offsets[3*i+j] = bodyCom[j];
			}
		}
	}
	static double Orig[3] = { 0.0, 0.0, 0.0 };	// Zero 
	double t[3];	// Translation from sdfast
	double dirCos[3][3];	// Direction cosines
	double com[3];

	for(int i=0;i<_model->getNB();i++) {
		// get position from sdfast
		// adjust by com
		_model->getBody(i)->getCenterOfMass(com);
		for(int k=0; k < 3; k++)
			Orig[k] = -_offsets[3*i+k];
		_model->getPosition(i, Orig, t);

		_model->getDirectionCosines(i,dirCos);
		// Initialize xform to identity
		_transforms[i].setPosition(t);
		_transforms[i].setRotationSubmatrix(dirCos);
	}
}

void SimtkAnimationCallback::getBodyRotations(int index, double rots[]) const
{
	for(int i=0; i<3; i++){
		rots[i] = _rotationAgles[3*index+i];
	}
}
