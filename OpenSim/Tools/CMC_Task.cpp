/* -------------------------------------------------------------------------- *
 *                           OpenSim:  CMC_Task.cpp                           *
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
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include "CMC_Task.h"
#include <OpenSim/Common/Function.h>

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTANTS
//=============================================================================
#define CENTER_OF_MASS_NAME string("center_of_mass")

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
CMC_Task::~CMC_Task()
{
}
//_____________________________________________________________________________
/**
 * Construct a default track object for a specified model.
 */
CMC_Task::CMC_Task() :
    TrackingTask(),
    _wrtBodyName(_propWRTBodyName.getValueStr()),
    _expressBodyName(_propExpressBodyName.getValueStr()),
    _active(_propActive.getValueBoolArray()),
    _kp(_propKP.getValueDblArray()),
    _kv(_propKV.getValueDblArray()),
    _ka(_propKA.getValueDblArray()),
    _r0(_propR0.getValueDblVec()),
    _r1(_propR1.getValueDblVec()),
    _r2(_propR2.getValueDblVec())
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTask Task object to be copied.
 */
CMC_Task::CMC_Task(const CMC_Task &aTask) :
    TrackingTask(aTask),
    _wrtBodyName(_propWRTBodyName.getValueStr()),
    _expressBodyName(_propExpressBodyName.getValueStr()),
    _active(_propActive.getValueBoolArray()),
    _kp(_propKP.getValueDblArray()),
    _kv(_propKV.getValueDblArray()),
    _ka(_propKA.getValueDblArray()),
    _r0(_propR0.getValueDblVec()),
    _r1(_propR1.getValueDblVec()),
    _r2(_propR2.getValueDblVec())
{
    setNull();
    copyData(aTask);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void CMC_Task::
setNull()
{
    setName(DEFAULT_NAME);
    setupProperties();

    _model = NULL;
    _wrtBodyName = "";
    _expressBodyName = "";
    //_on = true;
    _active[0] = _active[1] = _active[2] = false;
    //_w[0] = _w[1] = _w[2] = 1.0;
    _kp[0] = _kp[1] = _kp[2] = 1.0;
    _kv[0] = _kv[1] = _kv[2] = 0.5;
    _ka[0] = _ka[1] = _ka[2] = 1.0;
    _r0[0] = _r0[1] = _r0[2] = 0.0;
    _r1[0] = _r1[1] = _r1[2] = 0.0;
    _r2[0] = _r2[1] = _r2[2] = 0.0;
    _nTrk = 0;
    _pErrLast[0] = _pErrLast[1] = _pErrLast[2] = 0.0;
    _pErr[0] = _pErr[1] = _pErr[2] = 0.0;
    _vErrLast[0] = _vErrLast[1] = _vErrLast[2] = 0.0;
    _vErr[0] = _vErr[1] = _vErr[2] = 0.0;
    _aDes[0] = _aDes[1] = _aDes[2] = 0.0;
    _a[0] = _a[1] = _a[2] = 0.0;
    _j = NULL;
    _m = NULL;
}
//_____________________________________________________________________________
/**
 * Set up the properties.
 */
void CMC_Task::
setupProperties()
{
    _propWRTBodyName.setComment("Name of body frame with respect to which a tracking objective is specified. "
        "The special name '"+CENTER_OF_MASS_NAME+"' refers to the system center of mass. "
        "This property is not used for tracking joint angles.");
    _propWRTBodyName.setName("wrt_body");
    _propWRTBodyName.setValue("");
    _propertySet.append(&_propWRTBodyName);

    _propExpressBodyName.setComment("Name of body frame in which the tracking "
        "objectives are expressed.  This property is not used for tracking joint angles.");
    _propExpressBodyName.setName("express_body");
    _propExpressBodyName.setValue("");
    _propertySet.append(&_propExpressBodyName);

    Array<bool> active(false,3);
    _propActive.setComment("Array of 3 flags (each true or false) specifying whether a "
        "component of a task is active.  For example, tracking the trajectory of a point "
        "in space could have three components (x,y,z).  This allows each of those to be "
        "made active (true) or inactive (false).  A task for tracking a joint coordinate only "
        "has one component.");
    _propActive.setName("active");
    _propActive.setValue(active);
    _propertySet.append(&_propActive);

    Array<double> kp(1.0,3);
    _propKP.setComment("Position error feedback gain (stiffness). "
        "To achieve critical damping of errors, choose kv = 2*sqrt(kp).");
    _propKP.setName("kp");
    _propKP.setValue(kp);
    _propertySet.append(&_propKP);

    Array<double> kv(1.0,3);
    _propKV.setComment("Velocity error feedback gain (damping). "
        "To achieve critical damping of errors, choose kv = 2*sqrt(kp).");
    _propKV.setName("kv");
    _propKV.setValue(kv);
    _propertySet.append(&_propKV);

    Array<double> ka(1.0,3);
    _propKA.setComment("Feedforward acceleration gain.  "
        "This is normally set to 1.0, so no gain.");
    _propKA.setName("ka");
    _propKA.setValue(ka);
    _propertySet.append(&_propKA);

    Vec3 r(0.0);
    _propR0.setComment("Direction vector[3] for component 0 of a task. "
        "Joint tasks do not use this property.");
    _propR0.setName("r0");
    _propR0.setValue(r);
    _propertySet.append(&_propR0);

    _propR1.setComment("Direction vector[3] for component 1 of a task. "
        "Joint tasks do not use this property.");
    _propR1.setName("r1");
    _propR1.setValue(r);
    _propertySet.append(&_propR1);

    _propR2.setComment("Direction vector[3] for component 2 of a task. "
        "Joint tasks do not use this property.");
    _propR2.setName("r2");
    _propR2.setValue(r);
    _propertySet.append(&_propR2);
}

//_____________________________________________________________________________
/**
 * Copy the member data for this class only.
 *
 * @param aTask Object whose data is to be copied.
 */
void CMC_Task::
copyData(const CMC_Task &aTask)
{
    int i;
    _model = aTask.getModel();
    setWRTBodyName(aTask.getWRTBodyName());
    setExpressBodyName(aTask.getExpressBodyName());
    //setOn(aTask.getOn());
    for(i=0;i<3;i++) _active[i] = aTask.getActive(i);
    for(i=0;i<3;i++) _kp[i] = aTask.getKP(i);
    for(i=0;i<3;i++) _kv[i] = aTask.getKV(i);
    for(i=0;i<3;i++) _ka[i] = aTask.getKA(i);
    aTask.getDirection_0(_r0);
    aTask.getDirection_1(_r1);
    aTask.getDirection_2(_r2);

    // FUNCTIONS
    const Function *func;
    for(i=0;i<3;i++) {
        // position
        if(_pTrk[i]!=NULL) {delete _pTrk[i];  _pTrk[i]=NULL; }
        func = aTask.getTaskFunction(i);
        if(func!=NULL) _pTrk[i] = func->clone();
        // velocity
        if(_vTrk[i]!=NULL) {delete _vTrk[i];  _vTrk[i]=NULL; }
        func = aTask.getTaskFunctionForVelocity(i);
        if(func!=NULL) _vTrk[i] = func->clone();
        // acceleration
        if(_aTrk[i]!=NULL) {delete _aTrk[i];  _aTrk[i]=NULL; }
        func = aTask.getTaskFunctionForAcceleration(i);
        if(func!=NULL) _aTrk[i] = func->clone();
    }
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return  Reference to the altered object.
 */
CMC_Task& CMC_Task::
operator=(const CMC_Task &aTask)
{
    // BASE CLASS
    TrackingTask::operator =(aTask);

    // DATA
    copyData(aTask);

    return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// WRT BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the body with respect to (WRT) which the track goals are specified.
 *
 * @param aBody Body ID.
 */
void CMC_Task::
setWRTBodyName(std::string aBodyName)
{
    _wrtBodyName = aBodyName;
}
//_____________________________________________________________________________
/**
 * Get the body with respect to (WRT) which the track goals are specified.
 *
 * @return Body ID.
 */
std::string CMC_Task::
getWRTBodyName() const
{
    return(_wrtBodyName);
}

//-----------------------------------------------------------------------------
// EXPRESS BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the body in which the track goals are expressed.
 *
 * @param aBody Body ID.
 */
void CMC_Task::
setExpressBodyName(std::string aBodyName)
{
    _expressBodyName = aBodyName;
}
//_____________________________________________________________________________
/**
 * Get the body in which the track goals are expressed.
 *
 * @return Body ID.
 */
std::string CMC_Task::
getExpressBodyName() const
{
    return(_expressBodyName);
}

//-----------------------------------------------------------------------------
// ACTIVE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not track goals are active.
 *
 * @param a0 Active flag for track goal 0-- true means active.
 * @param a1 Active flag for track goal 1-- true means active.
 * @param a2 Active flag for track goal 2-- true means active.
 */
void CMC_Task::
setActive(bool a0,bool a1,bool a2)
{
    _active[0] = a0;
    _active[1] = a1;
    _active[2] = a2;
}
//_____________________________________________________________________________
/**
 * Get whether a specified track goal is active.
 *
 * @param aWhich Number of the track goal in question.
 * @return True if the specified track goal is active, false otherwise.
 */
bool CMC_Task::
getActive(int aWhich) const
{
    if(aWhich<0) return(false);
    if(aWhich>2) return(false);
    return(_active[aWhich]);
}

//-----------------------------------------------------------------------------
// WEIGHTS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the weight of each track goal.
 *
 * @param aW0 Weight for track goal 0.
 * @param aW1 Weight for track goal 1.
 * @param aW2 Weight for track goal 2.
 */
void CMC_Task::
setWeight(double aW0,double aW1,double aW2)
{
    _w[0] = aW0;
    _w[1] = aW1;
    _w[2] = aW2;
}
//_____________________________________________________________________________
/**
 * Get the weight of each track goal.
 *
 * @param aWhich Number of the track goal in question.
 * @return Weight.
 */
double CMC_Task::
getWeight(int aWhich) const
{
    if(aWhich<0) return(0.0);
    if(aWhich>2) return(0.0);
    return(_w[aWhich]);
}

//-----------------------------------------------------------------------------
// POSITION GAINS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the position gains for each track goal.
 *
 * @param aK0 Gain for track goal 0.
 * @param aK1 Gain for track goal 1.
 * @param aK2 Gain for track goal 2.
 */
void CMC_Task::
setKP(double aK0,double aK1,double aK2)
{
    _kp[0] = aK0;
    _kp[1] = aK1;
    _kp[2] = aK2;
}
//_____________________________________________________________________________
/**
 * Get the position gain for a specified track goal.
 *
 * @param aWhich Number of the track goal in question.
 * @return Position gain.
 */
double CMC_Task::
getKP(int aWhich) const
{
    if(aWhich<0) return(0.0);
    if(aWhich>2) return(0.0);
    return(_kp[aWhich]);
}

//-----------------------------------------------------------------------------
// VELOCITY GAINS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the velocity gains for each track goal.
 *
 * @param aK0 Gain for track goal 0.
 * @param aK1 Gain for track goal 1.
 * @param aK2 Gain for track goal 2.
 */
void CMC_Task::
setKV(double aK0,double aK1,double aK2)
{
    _kv[0] = aK0;
    _kv[1] = aK1;
    _kv[2] = aK2;
}
//_____________________________________________________________________________
/**
 * Get the velocity gain for a specified track goal.
 *
 * @param aWhich Number of the track goal in question.
 * @return Velocity gain.
 */
double CMC_Task::
getKV(int aWhich) const
{
    if(aWhich<0) return(0.0);
    if(aWhich>2) return(0.0);
    return(_kv[aWhich]);
}

//-----------------------------------------------------------------------------
// ACCELERATION GAINS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the acceleration gains for each track goal.
 *
 * @param aK0 Gain for track goal 0.
 * @param aK1 Gain for track goal 1.
 * @param aK2 Gain for track goal 2.
 */
void CMC_Task::
setKA(double aK0,double aK1,double aK2)
{
    _ka[0] = aK0;
    _ka[1] = aK1;
    _ka[2] = aK2;
}
//_____________________________________________________________________________
/**
 * Get the acceleration gain for a specified track goal.
 *
 * @param aWhich Number of the track goal in question.
 * @return Acceleration gain.
 */
double CMC_Task::
getKA(int aWhich) const
{
    if(aWhich<0) return(0.0);
    if(aWhich>2) return(0.0);
    return(_ka[aWhich]);
}

//-----------------------------------------------------------------------------
// DIRECTION 0
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the direction of track goal 0.
 *
 * @param aR Direction.  This vector is normalized.
 */
void CMC_Task::
setDirection_0(const SimTK::Vec3& aR)
{
    _r0 = aR; _r0.normalize();
}
//_____________________________________________________________________________
/**
 * Get the direction of track goal 0.
 *
 * @param aR Direction.
 */
void CMC_Task::
getDirection_0(SimTK::Vec3& rR) const
{
    rR=_r0;
}

//-----------------------------------------------------------------------------
// DIRECTION 1
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the direction of track goal 1.
 *
 * @param aR Direction.  This vector is normalized.
 */
void CMC_Task::
setDirection_1(const SimTK::Vec3& aR)
{
    _r1=aR; _r1.normalize();
}
//_____________________________________________________________________________
/**
 * Get the direction of track goal 1.
 *
 * @param aR Direction.
 */
void CMC_Task::
getDirection_1(SimTK::Vec3& rR) const
{
    rR=_r1;
}

//-----------------------------------------------------------------------------
// DIRECTION 2
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the direction of track goal 2.
 *
 * @param aR Direction.  This vector is normalized.
 */
void CMC_Task::
setDirection_2(const SimTK::Vec3& aR)
{
    _r2=aR;
}
//_____________________________________________________________________________
/**
 * Get the direction of track goal 2.
 *
 * @param aR Direction.
 */
void CMC_Task::
getDirection_2(SimTK::Vec3& rR) const
{
    rR=_r2;
}

//-----------------------------------------------------------------------------
// TRACK FUNCTIONS - POSITION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get a specified track function.
 *
 * @param aWhich Specifies which track function (0, 1, or 2).
 * @return Function.
 */
OpenSim::Function* CMC_Task::
getTaskFunction(int aWhich) const
{
    if(aWhich<0) return(NULL);
    if(aWhich>2) return(NULL);
    return(_pTrk[aWhich]);
}

//-----------------------------------------------------------------------------
// TRACK FUNCTIONS - VELOCITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the velocity track functions.  Note that this method makes copies of the
 * specified functions, so the caller may use the specified functions
 * for whatever purposes.
 *
 * @param aF0 Function for track goal 0.
 * @param aF1 Function for track goal 1.
 * @param aF2 Function for track goal 2.
 */
void CMC_Task::
setTaskFunctionsForVelocity(OpenSim::Function *aF0, OpenSim::Function *aF1, OpenSim::Function *aF2)
{
    if(_vTrk[0]!=NULL) { delete _vTrk[0];  _vTrk[0]=NULL; }
    if(_vTrk[1]!=NULL) { delete _vTrk[1];  _vTrk[1]=NULL; }
    if(_vTrk[2]!=NULL) { delete _vTrk[2];  _vTrk[2]=NULL; }

    if(aF0!=NULL) _vTrk[0] = aF0->clone();
    if(aF1!=NULL) _vTrk[1] = aF1->clone();
    if(aF2!=NULL) _vTrk[2] = aF2->clone();
}
//_____________________________________________________________________________
/**
 * Get a specified velocity track function.
 *
 * @param aWhich Specifies which track function (0, 1, or 2).
 * @return Function.
 */
OpenSim::Function* CMC_Task::
getTaskFunctionForVelocity(int aWhich) const
{
    if(aWhich<0) return(NULL);
    if(aWhich>2) return(NULL);
    return(_vTrk[aWhich]);
}

//-----------------------------------------------------------------------------
// TRACK FUNCTIONS - POSITION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the acceleration track functions.  Note that this method makes copies of
 * the specified track functions, so the caller may use the specified functions
 * for whatever purposes.
 *
 * @param aF0 Function for track goal 0.
 * @param aF1 Function for track goal 1.
 * @param aF2 Function for track goal 2.
 */
void CMC_Task::
setTaskFunctionsForAcceleration(
    OpenSim::Function *aF0, OpenSim::Function *aF1, OpenSim::Function *aF2)
{
    if(_aTrk[0]!=NULL) { delete _aTrk[0];  _aTrk[0]=NULL; }
    if(_aTrk[1]!=NULL) { delete _aTrk[1];  _aTrk[1]=NULL; }
    if(_aTrk[2]!=NULL) { delete _aTrk[2];  _aTrk[2]=NULL; }

    if(aF0!=NULL) _aTrk[0] = aF0->clone();
    if(aF1!=NULL) _aTrk[1] = aF1->clone();
    if(aF2!=NULL) _aTrk[2] = aF2->clone();
}
//_____________________________________________________________________________
/**
 * Get a specified acceleration track function.
 *
 * @param aWhich Specifies which track function (0, 1, or 2).
 * @return Function.
 */
OpenSim::Function* CMC_Task::
getTaskFunctionForAcceleration(int aWhich) const
{
    if(aWhich<0) return(NULL);
    if(aWhich>2) return(NULL);
    return(_aTrk[aWhich]);
}


//-----------------------------------------------------------------------------
// TASK KINEMATICS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the task position.
 *
 * @param aWhich Specifies which task goal (0, 1, or 2).
 * @param aT Time (in real time units).
 * @return Task position.
 * @throws Exception for an invalid task.
 */
double CMC_Task::
getTaskPosition(int aWhich,double aT) const
{
    if((aWhich<0)||(aWhich>=_nTrk)) {
        string msg = "CMC_Task: ERR- Invalid task.";
        throw( Exception(msg,__FILE__,__LINE__) );
    }
    double position = _pTrk[aWhich]->calcValue(SimTK::Vector(1,aT));
    return(position);
}
//_____________________________________________________________________________
/**
 * Get the task velocity.
 *
 * @param aWhich Specifies which task goal (0, 1, or 2).
 * @param aT Time (in real time units).
 * @return Task velocity.
 * @throws Exception for an invalid task.
 */
double CMC_Task::
getTaskVelocity(int aWhich,double aT) const
{
    if((aWhich<0)||(aWhich>=_nTrk)) {
        string msg = "CMC_Task: ERR- Invalid task.";
        throw( Exception(msg,__FILE__,__LINE__) );
    }

    double velocity;
    if(_vTrk[aWhich]!=NULL) {
        velocity = _vTrk[aWhich]->calcValue(SimTK::Vector(1,aT));
    } else {
        std::vector<int> derivComponents(1);
        derivComponents[0]=0;
        velocity = _pTrk[aWhich]->calcDerivative(derivComponents,SimTK::Vector(1,aT));
    }

    return( velocity );
}//_____________________________________________________________________________
/**
 * Get the task acceleration.
 *
 * @param aWhich Specifies which task goal (0, 1, or 2).
 * @param aT Time (in real time units).
 * @return Task acceleration.
 * @throws Exception for an invalid task.
 */
double CMC_Task::
getTaskAcceleration(int aWhich,double aT) const
{
    if((aWhich<0)||(aWhich>=_nTrk)) {
        string msg = "CMC_Task: ERR- Invalid task.";
        throw( Exception(msg,__FILE__,__LINE__) );
    }

    double acceleration;
    if(_aTrk[aWhich]!=NULL) {
        acceleration = _aTrk[aWhich]->calcValue(SimTK::Vector(1,aT));
    } else {
        std::vector<int> derivComponents(2);
        derivComponents[0]=0;
        derivComponents[1]=0;
        acceleration = _pTrk[aWhich]->calcDerivative(derivComponents,SimTK::Vector(1,aT));
    }

    return( acceleration );
}


//-----------------------------------------------------------------------------
// LAST POSITION ERRORS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the last achieved position error.  This information is useful for
 * checking that error dynamics are being followed.
 *
 * @param aE0 Last position error for track goal 0.
 * @param aE1 Last position error for track goal 1.
 * @param aE2 Last position error for track goal 2.
 */
void CMC_Task::
setPositionErrorLast(double aE0,double aE1,double aE2)
{
    _pErrLast[0] = aE0;
    _pErrLast[1] = aE1;
    _pErrLast[2] = aE2;
}
//_____________________________________________________________________________
/**
 * Get the last achieved position error.  This information is useful for
 * checking that error dynamics are being followed.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Last position error.
 */
double CMC_Task::
getPositionErrorLast(int aWhich) const
{
    if(aWhich<0) return(0.0);
    if(aWhich>2) return(0.0);
    return(_pErrLast[aWhich]);
}

//-----------------------------------------------------------------------------
// LAST VELOCITY ERRORS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the last achieved velocity error.  This information is useful for
 * checking that error dynamics are being followed.
 *
 * @param aE0 Last velocity error for track goal 0.
 * @param aE1 Last velocity error for track goal 1.
 * @param aE2 Last velocity error for track goal 2.
 */
void CMC_Task::
setVelocityErrorLast(double aE0,double aE1,double aE2)
{
    _vErrLast[0] = aE0;
    _vErrLast[1] = aE1;
    _vErrLast[2] = aE2;
}
//_____________________________________________________________________________
/**
 * Get the last achieved velocity error.  This information is useful for
 * checking that error dynamics are being followed.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Last velocity error.
 */
double CMC_Task::
getVelocityErrorLast(int aWhich) const
{
    if(aWhich<0) return(0.0);
    if(aWhich>2) return(0.0);
    return(_vErrLast[aWhich]);
}

//-----------------------------------------------------------------------------
// TRACK ERRORS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the position track error of a specified track goal.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Error
 */
double CMC_Task::
getPositionError(int aWhich) const
{
    if(aWhich<0) return(0.0);
    if(aWhich>2) return(0.0);
    return(_pErr[aWhich]);
}
//_____________________________________________________________________________
/**
 * Get the velocity track error of a specified track goal.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Error
 */
double CMC_Task::
getVelocityError(int aWhich) const
{
    if(aWhich<0) return(0.0);
    if(aWhich>2) return(0.0);
    return(_vErr[aWhich]);
}

//-----------------------------------------------------------------------------
// DESIRED ACCELERATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the desired acceleration of a specified track goal.
 * The method computeDesiredAccelerations() must be called first for the
 * values returned by this method to be valid.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Desired acceleration.  SimTK::NaN is returned on an error.
 */
double CMC_Task::
getDesiredAcceleration(int aWhich) const
{
    if(aWhich<0) return(SimTK::NaN);
    if(aWhich>2) return(SimTK::NaN);
    return(_aDes[aWhich]);
}

//-----------------------------------------------------------------------------
// ACCELERATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the acceleration of a specified track goal.  The acceleration returned
 * is the dot product of the appropriate track-goal direction and the
 * acceleration of the point or orientation in question.  In the case of
 * generalized coordinates, the acceleration of the generalized coordinate
 * is returned (i.e., a direction is not appropriate).
 *
 * For the value returned by this method to be valid, the method
 * computeAccelerations() must be called first.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Acceleration.  SimTK::NaN is returned on an error.
 */
double CMC_Task::
getAcceleration(int aWhich) const
{
    if(aWhich<0) return(SimTK::NaN);
    if(aWhich>2) return(SimTK::NaN);
    return(_a[aWhich]);
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//-----------------------------------------------------------------------------
// JACOBIAN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the Jacobian.
 */
void CMC_Task::
computeJacobian()
{
    log_error("CMC_Task::computeJacobian: This method should be overridden in "
        "derived classes.");
}

//-----------------------------------------------------------------------------
// EFFECTIVE MASS MATRIX
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the effective mass matrix.
 */
void CMC_Task::
computeEffectiveMassMatrix()
{
    log_error("CMC_Task::computeEffectiveMassMatrix: This method should be "
        "overridden in derived classes.");
}
