#ifndef CMC_TaskSet_h__
#define CMC_TaskSet_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  CMC_TaskSet.h                           *
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
#include <OpenSim/Common/FunctionSet.h>
#include "CMC_Task.h"

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * An class for holding and managing a set of tasks.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API CMC_TaskSet : public Set<TrackingTask> {
OpenSim_DECLARE_CONCRETE_OBJECT(CMC_TaskSet, Set<TrackingTask>);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
    /** Model for which the tracking is conducted. */
    Model *_model;
    /** Array of task positions. */
    Array<double> _pTask;
    /** Array of task velocities. */
    Array<double> _vTask;
    /** Array of task accelerations. */
    Array<double> _aTask;
    /** Array of last position errors. */
    Array<double> _pErrLast;
    /** Array of position errors. */
    Array<double> _pErr;
    /** Array of last velocity errors. */
    Array<double> _vErrLast;
    /** Array of velocity errors. */
    Array<double> _vErr;
    /** Array of gains for the position errors. */
    Array<double> _kp;
    /** Array of gains for the velocity errors. */
    Array<double> _kv;
    /** Array of gains for the acceleration errors. */
    Array<double> _ka;
    /** Array of weights of the desired acceleration. */
    Array<double> _w;
    /** Array of desired accelerations. */
    Array<double> _aDes;
    /** Array of accelerations. */
    Array<double> _a;

    /** In case there are tracking targets loaded from file, the filename goes here
     * and the column names go into individual targets.
     */
    PropertyStr _dataFileNameProp;
    std::string &_dataFileName;

    FunctionSet  _functions;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    CMC_TaskSet();
    explicit CMC_TaskSet(const std::string &aFileName);
    virtual ~CMC_TaskSet();

    CMC_TaskSet& operator=(const CMC_TaskSet&) {
        throw OpenSim::Exception("CMC_TaskSet::operator=() not implemented");
        return *this;
    }
    CMC_TaskSet(const CMC_TaskSet& aCMCTaskSet) :
        Set<TrackingTask>(aCMCTaskSet),
        _dataFileName(_dataFileNameProp.getValueStr()) {
        _propertySet = aCMCTaskSet.getPropertySet();
    }

private:
    void setNull();
    void setupProperties();

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
public:
    // MODEL
    void setModel(Model& aModel);
    Model* getModel() const;

    const std::string& getDataFileName() const { return _dataFileName; };
    // FUNCTIONS
    void setFunctions(FunctionSet &aFuncSet);
    void setFunctionsForVelocity(FunctionSet &aFuncSet);
    void setFunctionsForAcceleration(FunctionSet &aFuncSet);
    int getNumActiveTaskFunctions() const;
    // KINEMATICS
    Array<double>& getTaskPositions(double aT);
    Array<double>& getTaskVelocities(double aT);
    Array<double>& getTaskAccelerations(double aT);
    // GAINS
    Array<double>& getPositionGains();
    Array<double>& getVelocityGains();
    Array<double>& getAccelerationGains();
    // ERRORS
    Array<double>& getPositionErrorsLast();
    Array<double>& getPositionErrors();
    Array<double>& getVelocityErrorsLast();
    Array<double>& getVelocityErrors();
    // WEIGHTS
    Array<double>& getWeights();
    // DESIRED ACCELERATIONS
    Array<double>& getDesiredAccelerations();
    // ACCELERATIONS
    Array<double>& getAccelerations();

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    void recordErrorsAsLastErrors();
    void computeErrors(const SimTK::State& s, double aT);
    void computeDesiredAccelerations(const SimTK::State& s, double aT);
    void computeDesiredAccelerations(const SimTK::State& s, double aTCurrent,double aTFuture);
    void computeAccelerations(const SimTK::State& s );


//=============================================================================
};  // END of class CMC_TaskSet
//=============================================================================
//=============================================================================

};  // end namespace

#endif // CMC_TaskSet_h__


