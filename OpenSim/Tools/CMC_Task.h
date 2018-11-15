#ifndef CMC_Task_h__
#define CMC_Task_h__
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  CMC_Task.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

// INCLUDES
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyBoolArray.h>
#include "TrackingTask.h"

namespace SimTK {
class State;
}

namespace OpenSim {

class Function;

//=============================================================================
//=============================================================================
/**
 * An abstract base class for specifying a task objective for
 * a dynamic simulation.  This class supports joint, point, and orientation
 * task objectives.  Specific implementations for these kinds of control
 * tasks should inherit from this class.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API CMC_Task : public TrackingTask {
OpenSim_DECLARE_ABSTRACT_OBJECT(CMC_Task, TrackingTask);

//=============================================================================
// DATA
//=============================================================================
protected:
    // PROPERTIES
    /** Body with respect to which the task goals are specified. */
    PropertyStr _propWRTBodyName;
    /** Body frame in which the task goals are expressed. */
    PropertyStr _propExpressBodyName;
    /** Property to specify the active task goals. */
    PropertyBoolArray _propActive;
    /** Position error feedback gain. */
    PropertyDblArray _propKP;
    /** Velocity error feedback gain. */
    PropertyDblArray _propKV;
    /** Feedforward acceleration gain. */
    PropertyDblArray _propKA;
    /** Directions of the task goal 0. */
    PropertyDblVec3 _propR0;
    /** Directions of the task goal 1. */
    PropertyDblVec3 _propR1;
    /** Directions of the task goal 2. */
    PropertyDblVec3 _propR2;

    // REFERENCES TO PROPERTY VALUES
    // NOTE- These reference variables must be listed in the class after
    // the properties to which they refer.  The order in which member
    // variables are listed determines the order in which the member
    // variables are initialized.  The properties must be initialized
    // before the references can be initialized to something meaningful.
    /** Reference to the value of the on property. */
    //bool &_on;
    /** Reference to the value of the WRTBody property. */
    std::string &_wrtBodyName;
    /** Reference to the value of the ExpressBody property. */
    std::string &_expressBodyName;
    /** Reference to the value of the Active property. */
    Array<bool> &_active;
    /** Reference to the value of the KP property. */
    Array<double> &_kp;
    /** Reference to the value of the KV property. */
    Array<double> &_kv;
    /** Reference to the value of the KA property. */
    Array<double> &_ka;
    /** Reference to the value of the R0 property. */
    SimTK::Vec3 &_r0;
    /** Reference to the value of the R1 property. */
    SimTK::Vec3 &_r1;
    /** Reference to the value of the R2 property. */
    SimTK::Vec3 &_r2;

    /** Last position error. */
    SimTK::Vec3 _pErrLast;
    /** Position error. */
    SimTK::Vec3 _pErr;
    /** Last velocity error. */
    SimTK::Vec3 _vErrLast;
    /** Velocity error. */
    SimTK::Vec3 _vErr;
    /** Desired accelerations. */
    SimTK::Vec3 _aDes;
    /** Accelerations. */
    SimTK::Vec3 _a;
    /** Jacobian. */
    double *_j;
    /** Effective mass matrix. */
    double *_m;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    CMC_Task();
    CMC_Task(const CMC_Task &aTaskObject);
    virtual ~CMC_Task();

private:
    void setNull();
    void setupProperties();
    void copyData(const CMC_Task &aTaskObject);

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:

#ifndef SWIG
    CMC_Task& operator=(const CMC_Task &aTaskObject);
#endif

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    // WRT BODY
    void setWRTBodyName(std::string aBodyName);
    std::string getWRTBodyName() const;
    // EXPRESS BODY
    void setExpressBodyName(std::string aBodyName);
    std::string getExpressBodyName() const;
    // ACTIVE
    void setActive(bool a0,bool a1=false,bool a2=false);
    bool getActive(int aWhich) const;
    // WEIGHTS
    void setWeight(double aW0,double aW1=0.0,double aW2=0.0);
    double getWeight(int aWhich) const;
    // POSITION FEEDBACK GAINS
    void setKP(double aK0,double aK1=0.0,double aK2=0.0);
    double getKP(int aWhich) const;
    // VELOCITY FEEDBACK GAINS
    void setKV(double aK0,double aK1=0.0,double aK2=0.0);
    double getKV(int aWhich) const;
    // ACCELERATION FEEDFORWARD GAINS
    void setKA(double aK0,double aK1=0.0,double aK2=0.0);
    double getKA(int aWhich) const;
    // DIRECTION OF TASK 0
    void setDirection_0(const SimTK::Vec3& aR);
    void getDirection_0(SimTK::Vec3& rR) const;
    // DIRECTION OF TASK 1
    void setDirection_1(const SimTK::Vec3& aR);
    void getDirection_1(SimTK::Vec3& rR) const;
    // DIRECTION OF TASK 2
    void setDirection_2(const SimTK::Vec3& aR);
    void getDirection_2(SimTK::Vec3& rR) const;
    // TASK FUNCTIONS
    Function* getTaskFunction(int aWhich) const;
    void setTaskFunctionsForVelocity(Function *aF0,
        Function *aF1=NULL,Function *aF2=NULL);
    Function* getTaskFunctionForVelocity(int aWhich) const;
    void setTaskFunctionsForAcceleration(Function *aF0,
        Function *aF1=NULL,Function *aF2=NULL);
    Function* getTaskFunctionForAcceleration(int aWhich) const;
    // TASK KINEMATICS
    double getTaskPosition(int aWhich,double aT) const;
    double getTaskVelocity(int aWhich,double aT) const;
    double getTaskAcceleration(int aWhich,double aT) const;
    // LAST ERRORS
    void setPositionErrorLast(double aE0,double aE1=0.0,double aE2=0.0);
    double getPositionErrorLast(int aWhich) const;
    void setVelocityErrorLast(double aE0,double aE1=0.0,double aE2=0.0);
    double getVelocityErrorLast(int aWhich) const;
    // ERRORS
    double getPositionError(int aWhich) const;
    double getVelocityError(int aWhich) const;
    // DESIRED ACCELERATIONS
    double getDesiredAcceleration(int aWhich) const;
    // ACCELERATIONS
    double getAcceleration(int aWhich) const;

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    virtual void computeErrors(const SimTK::State& s, double aT) = 0;
    virtual void computeDesiredAccelerations(const SimTK::State& s, double aT) = 0;
    virtual void computeDesiredAccelerations(const SimTK::State& s, double aTI,double aTF) = 0;
    virtual void computeAccelerations(const SimTK::State& s ) = 0;
    virtual void computeJacobian();
    virtual void computeEffectiveMassMatrix();

//=============================================================================
};  // END of class CMC_Task
//=============================================================================
//=============================================================================

}; // end namespace

#endif // __CMC_Task_h__


