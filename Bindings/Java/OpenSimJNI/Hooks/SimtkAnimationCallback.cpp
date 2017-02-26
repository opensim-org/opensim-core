/* -------------------------------------------------------------------------- *
 *                    OpenSim:  SimtkAnimationCallback.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/*  
 * Author:  
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "SimtkAnimationCallback.h"


using namespace std;
using namespace OpenSim;

//Synchronization stuff 
//int SimtkAnimationCallback::_turn = 1;
//bool SimtkAnimationCallback::_flag[] = {false, false};
//=============================================================================
// STATICS
//=============================================================================

/**
 * Create an animation callback associated with the passed in model, add it to
 * the model and return it.
 * Callbacks are auto deleted by the model destructor.
 */
SimtkAnimationCallback*
SimtkAnimationCallback::CreateAnimationCallback(Model *aModel)
{
    SimtkAnimationCallback* newAnimationCallback = new SimtkAnimationCallback(aModel);
    aModel->addIntegCallback(newAnimationCallback);
    return newAnimationCallback;
}

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimtkAnimationCallback::~SimtkAnimationCallback()
{
    delete[] _transforms;
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
SimtkAnimationCallback::SimtkAnimationCallback(Model *aModel, Model *aModelForDisplay) :
    IntegCallback(aModel)
{
    //cout<<"\n\nCreating new SimtkAnimationCallback...\n";
    // NULL
    setNull();

    if(aModelForDisplay) _modelForDisplay = aModelForDisplay;
    else _modelForDisplay = aModel;

    AbstractDynamicsEngine& de = getModelForDisplay()->getDynamicsEngine();
    // We could keep pointer to body's xform, but that requires display
    // to be synchronous.
    // For now we make new list, copy then we'll fix performance
    _transforms = new Transform[de.getNumBodies()];

    // Make sure the models are compatible (in terms of q's and u's)
    assert(getModel()->getDynamicsEngine().getNumCoordinates() == getModelForDisplay()->getDynamicsEngine().getNumCoordinates());
    assert(getModel()->getDynamicsEngine().getNumSpeeds() == getModelForDisplay()->getDynamicsEngine().getNumSpeeds());

    if(getModel()!=getModelForDisplay()) {
        Array<std::string> stateNames1, stateNames2;
        getModel()->getStateNames(stateNames1);
        getModelForDisplay()->getStateNames(stateNames2);
        if(stateNames1==stateNames2) _modelForDisplayCompatibleStates=true;
    }

    getTransformsFromKinematicsEngine(*getModelForDisplay());

    _currentTime=0.0;
    //cout<<endl<<endl;
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
    _modelForDisplaySetConfiguration=true;
    _modelForDisplayCompatibleStates=false;
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
step(double *aXPrev,double *aYPrev,double *aYPPrev,int aStep,double aDT,double aT,
        double *aX,double *aY,double *aYP,double *aDYDT,void *aClientData)
{
    _currentTime = aT;

    if(!proceed(aStep)) return 0;

    if(getModelForDisplay() != getModel() && _modelForDisplaySetConfiguration) {
        if(_modelForDisplayCompatibleStates) getModelForDisplay()->setStates(aY);
        else getModelForDisplay()->getDynamicsEngine().setConfiguration(&aY[0], &aY[getModelForDisplay()->getNumCoordinates()]);
    }
    
    //mutex_begin(0);   // Intention is to make sure xforms that are cached are consistent from the same time
    getTransformsFromKinematicsEngine(*getModelForDisplay());
    // update muscle geometry
    getModelForDisplay()->getActuatorSet()->updateDisplayers();
    //mutex_end(0);
    return (0);
}

const Transform* SimtkAnimationCallback::getBodyTransform(int index) const
{
    return &_transforms[index];
}
/**
 * Cache Coms for bodies so that we can get the xform for the bodies from SDFast in one go with 
 * minimal computation on our side. If displaying a SimmModel for example in IK then offsets are set to 0
 */
void SimtkAnimationCallback::extractOffsets(Model& displayModel)
{
}
/*------------------------------------------------------------------
 * getTransformsFromKinematicsEngine is a utility used to filll the 
 * _transforms array from a SimmKinematicsEngine
 * @param dModel: The model to use with associated kinematicsEngine
 */
void SimtkAnimationCallback::
getTransformsFromKinematicsEngine(Model& dModel)
{
    AbstractDynamicsEngine& de = dModel.getDynamicsEngine();
    BodySet *bodySet = de.getBodySet();
    AbstractBody* body;
    int i,nb = bodySet->getSize();
    for(i=0;i<nb;i++) {
        body = bodySet->get(i);
//  JACKM NEEDS STATE       _transforms[i] = de.getTransform(*body);
    }
}
