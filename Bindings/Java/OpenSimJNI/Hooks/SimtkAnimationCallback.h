#ifndef _SimtkAnimationCallback_h_
#define _SimtkAnimationCallback_h_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  SimtkAnimationCallback.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ayman Habib                                  *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Transform.h>
#include <OpenSim/Simulation/Model/IntegCallback.h>
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
//=============================================================================
/**
 * A class for generating an animation sequence in Simtk platform based on a
 * forward dynamic simulation.
 *
 * @author Ayman Habib (After rdmAnimationCallback)
 * @version 1.0
 */
namespace OpenSim { 

class SimtkAnimationCallback : public IntegCallback
{
//=============================================================================
// DATA
//=============================================================================
protected:
    
    Transform*      _transforms;
    /** Current simulation time for feedback purposes */
    double          _currentTime;
    double*         _offsets;

    Model*          _modelForDisplay; // might be a different model than the one we're the callback on
    bool                _modelForDisplaySetConfiguration;
    bool                _modelForDisplayCompatibleStates;
//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    // A Factory method to create the callback, so that it's created on the C++ side
    // and memory menagement on the Java side is side-stepped
    static SimtkAnimationCallback* CreateAnimationCallback(Model *aModel);
    virtual void migrateFromPreviousVersion(const Object* aObject) { return; };
protected:
    SimtkAnimationCallback(Model *aModel, Model *aModelForDisplay=0);
    virtual ~SimtkAnimationCallback();
private:
    void setNull();
public:

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    const double getCurrentTime() const;
    //--------------------------------------------------------------------------
    // CALLBACKS
    //--------------------------------------------------------------------------
    virtual int
        step(double *aXPrev,double *aYPrev,double *aYPPrev,int aStep,double aDT,double aT,
        double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
    virtual int
        begin(int aStep,double aDT,double aT,
        double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL)
    {
        _currentTime=aDT;
        return 0;
    }
    const Transform* getBodyTransform(int bodyIndex) const;

    void extractOffsets(Model& displayModel);

    Model *getModelForDisplay() { return _modelForDisplay; }
    void setModelForDisplaySetConfiguration(bool aSetConfiguration) { _modelForDisplaySetConfiguration = aSetConfiguration; }
    bool getModelForDisplayCompatibleStates() { return _modelForDisplayCompatibleStates; }
public:
    // Load transforms vector from KinematicsEngine
    void getTransformsFromKinematicsEngine(Model& simmModel);
    // Synchronization stuff
    // Implementation of Peterson's Algorithm that doesn't work!
/*
    static int _turn;
    static bool _flag[2];
public:
    void mutex_begin(int i) { 
        int j = 1-i; // i=me, j=other 
        _flag[i] = true;                         
        _turn = j;                               
        //std::cout << "In mutex_begin:" << i << j 
        //  << _flag[0] << _flag[1] 
        //  << _turn << std::endl;
        while (_flag[j] && _turn==j) ;// skip 
        //std::cout << "Leave mutex_begin:" << i << std::endl;

    }
    void mutex_end(int i){
        _flag[i] = false;                         
    }
*/
//=============================================================================
};  // END of class SimtkAnimationCallback

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimtkAnimationCallback_h__


