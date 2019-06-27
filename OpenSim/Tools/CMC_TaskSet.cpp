/* -------------------------------------------------------------------------- *
 *                         OpenSim:  CMC_TaskSet.cpp                          *
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
#include "CMC_TaskSet.h"
#include "StateTrackingTask.h"


using namespace std;
using namespace OpenSim;

#ifndef SWIG
template class OSIMTOOLS_API OpenSim::Set<OpenSim::CMC_Task>;
#endif

//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
CMC_TaskSet::~CMC_TaskSet()
{

}
//_____________________________________________________________________________
/**
 * Construct a default task set for a specified model.
 *
 * @param aModel Model for which tasks will be set.
 */
CMC_TaskSet::CMC_TaskSet() :
    _pTask(0.0), 
    _vTask(0.0),
    _aTask(0.0),
    _pErrLast(0.0),
    _pErr(0.0),
    _vErrLast(0.0),
    _vErr(0.0),
    _kp(0.0),
    _kv(0.0),
    _ka(0.0),
    _w(0.0),
    _aDes(0.0),
    _a(0.0),
    _dataFileName(_dataFileNameProp.getValueStr())
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Construct a task set from file.
 *
 * @param aFileName Name of the file.
 */
CMC_TaskSet::CMC_TaskSet(const string &aFileName) :
    Set<TrackingTask>(aFileName, false),
    _pTask(0.0),
    _vTask(0.0),
    _aTask(0.0),
    _pErrLast(0.0),
    _pErr(0.0),
    _vErrLast(0.0),
    _vErr(0.0),
    _kp(0.0),
    _kv(0.0),
    _ka(0.0),
    _w(0.0),
    _aDes(0.0),
    _a(0.0),
    _dataFileName(_dataFileNameProp.getValueStr())
{
    setNull();
    updateFromXMLDocument();
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void CMC_TaskSet::
setNull()
{
    setupProperties();

    _model = NULL;
    _pErrLast.setSize(0);
    _pErr.setSize(0);
    _vErrLast.setSize(0);
    _vErr.setSize(0);
    _w.setSize(0);
    _aDes.setSize(0);
    _a.setSize(0);
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.
 */
void CMC_TaskSet::
setupProperties()
{
    _dataFileNameProp.setName("datafile");
    _dataFileName="";
    _propertySet.append(&_dataFileNameProp);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the model for which the tracking is performed.  The model is set
 * for all tasks that are currently being managed by this set.
 *
 * @param aModel Model.
 */
void CMC_TaskSet::
setModel(Model& aModel)
{
    _model = &aModel;

    int i;
    for(i=0;i<getSize();i++) {
        TrackingTask& task = get(i);
        task.setModel(*_model);
    }
}

//_____________________________________________________________________________
/**
 * Get the model for which the tracking is performed.
 *
 * @return Pointer to the model.
 */
Model* CMC_TaskSet::
getModel() const
{
    return(_model);
}

//-----------------------------------------------------------------------------
// PARAMETERS BASED ON NAME
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// FUNCTION SET - POSITIONS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the functions for the tasks.  Functions are set based on the
 * correspondence of the function and the task.  For example,
 * a task with the name "x" will search for a function or functions
 * with the name "x".  For tasks that require 3 functions, such
 * as CMC_Point tasks, the assumption is that there will be three
 * consecutive functions named "x" in the function set.  If the correct
 * number of functions is not found, the task is disabled.
 *
 * @param aFuncSet Function set.
 * @return Pointer to the previous function set.
 */
void CMC_TaskSet::setFunctions(FunctionSet &aFuncSet)
{
    // LOOP THROUGH TRACK OBJECTS
    int i,j,iFunc=0;
    int nTrk;
    string name;
    Function *f[3];
    for(i=0;i<getSize();i++) {

        // OBJECT
        TrackingTask& ttask = get(i);

        if (dynamic_cast<StateTrackingTask*>(&ttask)!=NULL) {
            StateTrackingTask& sTask = dynamic_cast<StateTrackingTask&>(ttask);
            if (aFuncSet.contains(sTask.getName())){
                sTask.setTaskFunctions(&aFuncSet.get(sTask.getName()));
            }
            else{
                cout << "State tracking task " << sTask.getName() 
                    << "has no data to track and will be ignored" << std::endl;
            }
            continue;
        }
        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        // NAME
        name = task.getName();
        if(name.empty()) continue;

        // FIND FUNCTION(S)
        f[0] = f[1] = f[2] = NULL;
        nTrk = task.getNumTaskFunctions();
        iFunc = aFuncSet.getIndex(name,iFunc);
        if (iFunc < 0){
            const Coordinate& coord = _model->getCoordinateSet().get(name);
            name = coord.getStateVariableNames()[0];
            iFunc = aFuncSet.getIndex(name, iFunc);
            if (iFunc < 0){
                string msg = "CMC_TaskSet::setFunctions: function for task '";
                msg += name + " not found.";
                throw Exception(msg);
            }
        }

        for(j=0;j<nTrk;j++) {
            try {
                f[j] = &aFuncSet.get(iFunc);
            } catch(const Exception& x) {
                x.print(cout);
            }
            if(f[j]==NULL) break;
            if(name == f[j]->getName()) {
                iFunc++;
            } else {
                f[j] = NULL;
                break;
            }
        }
        task.setTaskFunctions(f[0],f[1],f[2]);
    }
}

//-----------------------------------------------------------------------------
// FUNCTION SET - VELOCITIES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the velocity functions for the tasks.  Functions are set based on the
 * correspondence of the function and the task.  For example,
 * a task with the name "x" will search for a function or functions
 * with the name "x".  For tasks that require 3 functions, such
 * as CMC_Point tasks, the assumption is that there will be three
 * consecutive functions named "x" in the function set.  If the correct
 * number of functions is not found, the task is disabled.
 *
 * @param aFuncSet Function set.
 * @return Pointer to the previous function set.
 */
void CMC_TaskSet::
setFunctionsForVelocity(FunctionSet &aFuncSet)
{
    // LOOP THROUGH TRACK OBJECTS
    int i,j,iFunc=0;
    int nTrk;
    string name;
    Function *f[3];

    const CoordinateSet& coords = getModel()->getCoordinateSet(); 

    for(i=0;i<getSize();i++) {

        // OBJECT
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);

        // NAME
        name = task.getName();
        if(name.empty()) continue;

        const Coordinate& coord = coords.get(name);

        // FIND FUNCTION(S)
        f[0] = f[1] = f[2] = NULL;
        nTrk = task.getNumTaskFunctions();
        iFunc = aFuncSet.getIndex(coord.getSpeedName(),iFunc);

        if (iFunc < 0){
            const Coordinate& coord = _model->getCoordinateSet().get(name);
            name = coord.getStateVariableNames()[1]; // index 1 is speed
            iFunc = aFuncSet.getIndex(name, iFunc);
            if (iFunc < 0){
                string msg = "CMC_TaskSet::setFunctionsForVelocity: function for task '";
                msg += name + " not found.";
                throw Exception(msg);
            }
        }

        for(j=0;j<nTrk;j++) {
            try {
                f[j] = &aFuncSet.get(iFunc);
            } catch(const Exception& x) {
                x.print(cout);
            }
            if(f[j]==NULL) break;
        }
        task.setTaskFunctionsForVelocity(f[0],f[1],f[2]);
    }
}

//-----------------------------------------------------------------------------
// FUNCTION SET - QUERY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Return number of active task functions.
 */
int CMC_TaskSet::
getNumActiveTaskFunctions() const
{
    int count=0;
    for(int i=0; i<getSize(); i++) {
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        for(int j=0; j<task.getNumTaskFunctions(); j++)
            if(task.getActive(j)) count++;
    }
    return count;
}


//-----------------------------------------------------------------------------
// FUNCTION SET - ACCELERATIONS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the acceleration functions for the tasks.  Functions are set based on
 * the correspondence of the function and the task.  For example,
 * a task with the name "x" will search for a function or functions
 * with the name "x".  For tasks that require 3 functions, such
 * as CMC_Point tasks, the assumption is that there will be three
 * consecutive functions named "x" in the function set.  If the correct
 * number of functions is not found, the task is disabled.
 *
 * @param aFuncSet Function set.
 * @return Pointer to the previous function set.
 */
void CMC_TaskSet::
setFunctionsForAcceleration(FunctionSet &aFuncSet)
{
    // LOOP THROUGH TRACK OBJECTS
    int i,j,iFunc=0;
    int nTrk;
    string name;
    Function *f[3];

    const CoordinateSet& coords = getModel()->getCoordinateSet(); 

    for(i=0;i<getSize();i++) {

        // OBJECT
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        // NAME
        name = task.getName();
        if(name.empty()) continue;

        const Coordinate& coord = coords.get(name);

        // FIND FUNCTION(S)
        f[0] = f[1] = f[2] = NULL;
        nTrk = task.getNumTaskFunctions();
        iFunc = aFuncSet.getIndex(coord.getSpeedName(),iFunc);

        if (iFunc < 0){
            const Coordinate& coord = _model->getCoordinateSet().get(name);
            name = coord.getStateVariableNames()[1];
            iFunc = aFuncSet.getIndex(name, iFunc);
            if (iFunc < 0){
                string msg = "CMC_TaskSet::setFunctionsForAcceleration: function for task '";
                msg += name + " not found.";
                throw Exception(msg);
            }
        }

        for(j=0;j<nTrk;j++) {
            try {
                f[j] = &aFuncSet.get(iFunc);
            } catch(const Exception& x) {
                x.print(cout);
            }
            if(f[j]==NULL) break;
        }
        task.setTaskFunctionsForAcceleration(f[0],f[1],f[2]);
    }
}

//-----------------------------------------------------------------------------
// KINEMATICS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get all active task positions.
 *
 * @param aT Time (in real time units).
 * @return Array of task positions.
 */
OpenSim::Array<double>& CMC_TaskSet::
getTaskPositions(double aT)
{
    _pTask.setSize(0);

    int i,j,n;
    int size = getSize();
    for(i=0;i<size;i++) {
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        n = task.getNumTaskFunctions();
        for(j=0;j<n;j++) {
            if(!task.getActive(j)) continue;
            _pTask.append(task.getTaskPosition(j,aT));
        }
    }

    return(_pTask);
}
//_____________________________________________________________________________
/**
 * Get all active task velocities.
 *
 * @param aT Time (in real time units).
 * @return Array of task velocities.
 */
OpenSim::Array<double>& CMC_TaskSet::
getTaskVelocities(double aT)
{
    _vTask.setSize(0);

    int i,j,n;
    int size = getSize();
    for(i=0;i<size;i++) {
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        n = task.getNumTaskFunctions();
        for(j=0;j<n;j++) {
            if(!task.getActive(j)) continue;
            _vTask.append(task.getTaskVelocity(j,aT));
        }
    }

    return(_vTask);
}
//_____________________________________________________________________________
/**
 * Get all active task accelerations.
 *
 * @param aT Time (in real time units).
 * @return Array of task accelerations.
 */
OpenSim::Array<double>& CMC_TaskSet::
getTaskAccelerations(double aT)
{
    _aTask.setSize(0);

    int i,j,n;
    int size = getSize();
    for(i=0;i<size;i++) {
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        n = task.getNumTaskFunctions();
        for(j=0;j<n;j++) {
            if(!task.getActive(j)) continue;
            _aTask.append(task.getTaskAcceleration(j,aT));
        }
    }

    return(_aTask);
}

//-----------------------------------------------------------------------------
// GAINS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get all active gains for position errors.
 *
 * @param aT Time (in real time units).
 * @return Array of position gains.
 */
OpenSim::Array<double>& CMC_TaskSet::
getPositionGains()
{
    _kp.setSize(0);

    int i,j,n;
    int size = getSize();
    for(i=0;i<size;i++) {
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        n = task.getNumTaskFunctions();
        for(j=0;j<n;j++) {
            if(!task.getActive(j)) continue;
            _kp.append(task.getKP(j));
        }
    }

    return(_kp);
}
//_____________________________________________________________________________
/**
 * Get all active gains for velocity errors.
 *
 * @param aT Time (in real time units).
 * @return Array of velocity gains.
 */
OpenSim::Array<double>& CMC_TaskSet::
getVelocityGains()
{
    _kv.setSize(0);

    int i,j,n;
    int size = getSize();
    for(i=0;i<size;i++) {
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        n = task.getNumTaskFunctions();
        for(j=0;j<n;j++) {
            if(!task.getActive(j)) continue;
            _kv.append(task.getKV(j));
        }
    }

    return(_kv);
}
//_____________________________________________________________________________
/**
 * Get all active gains for acceleration errors.
 *
 * @param aT Time (in real time units).
 * @return Array of velocity gains.
 */
OpenSim::Array<double>& CMC_TaskSet::
getAccelerationGains()
{
    _ka.setSize(0);

    int i,j,n;
    int size = getSize();
    for(i=0;i<size;i++) {
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        n = task.getNumTaskFunctions();
        for(j=0;j<n;j++) {
            if(!task.getActive(j)) continue;
            _ka.append(task.getKA(j));
        }
    }

    return(_ka);
}

//-----------------------------------------------------------------------------
// ERRORS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the last position errors.
 * @return Array of last position errors.
 */
OpenSim::Array<double>& CMC_TaskSet::
getPositionErrorsLast()
{
    return(_pErrLast);
}
//_____________________________________________________________________________
/**
 * Get the position errors.
 * @return Array of position errors.
 */
OpenSim::Array<double>& CMC_TaskSet::
getPositionErrors()
{
    return(_pErr);
}
//_____________________________________________________________________________
/**
 * Get the last velocity errors.
 * @return Array of last velocity errors.
 */
OpenSim::Array<double>& CMC_TaskSet::
getVelocityErrorsLast()
{
    return(_vErrLast);
}
//_____________________________________________________________________________
/**
 * Get the velocity errors.
 * @return Array of velocity errors.
 */
OpenSim::Array<double>& CMC_TaskSet::
getVelocityErrors()
{
    return(_vErr);
}

//-----------------------------------------------------------------------------
// WEIGHTS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the array of weights for the desired accelerations.
 * @return Array of weights.
 */
OpenSim::Array<double>& CMC_TaskSet::
getWeights()
{
    _w.setSize(0);

    int i,j,n;
    int size = getSize();
    for(i=0;i<size;i++) {
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        n = task.getNumTaskFunctions();
        for(j=0;j<n;j++) {
            if(!task.getActive(j)) continue;
            _w.append(task.getWeight(j));
        }
    }

    return(_w);
}

//-----------------------------------------------------------------------------
// DESIRED ACCELERATIONS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the desired accelerations.
 * @return Array of desired accelerations.
 */
OpenSim::Array<double>& CMC_TaskSet::
getDesiredAccelerations()
{
    return(_aDes);
}

//-----------------------------------------------------------------------------
// ACCELERATIONS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the accelerations.
 * @return Array of accelerations.
 */
OpenSim::Array<double>& CMC_TaskSet::
getAccelerations()
{
    return(_a);
}



//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the current position errors of all tasks as the last errors
 * that were achieved during a simulation.
 *
 * This method assumes that the position errors have already been computed.
 */
void CMC_TaskSet::
recordErrorsAsLastErrors()
{
    _pErrLast.setSize(0);
    _vErrLast.setSize(0);

    int i,j;
    double e0,e1,e2;
    for(i=0;i<getSize();i++) {

        // OBJECT
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        
        // POSITION ERRORS
        e0 = task.getPositionError(0);
        e1 = task.getPositionError(1);
        e2 = task.getPositionError(2);
        task.setPositionErrorLast(e0,e1,e2);

        // VELOCITY ERRORS
        e0 = task.getVelocityError(0);
        e1 = task.getVelocityError(1);
        e2 = task.getVelocityError(2);
        task.setVelocityErrorLast(e0,e1,e2);

        // SET ERRORS
        for(j=0;j<3;j++) {
            if(!task.getActive(j)) continue;
            _pErrLast.append(task.getPositionErrorLast(j));
            _vErrLast.append(task.getVelocityErrorLast(j));
        }
    }
}
//_____________________________________________________________________________
/**
 * Compute the errors for all tasks.
 *
 * @param aT Time at which to compute the errors in real time units.
 */
void CMC_TaskSet::
computeErrors(const SimTK::State& s, double aT)
{
    _pErr.setSize(0);
    _vErr.setSize(0);

    int i,j;
    for(i=0;i<getSize();i++) {

        // OBJECT
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        
        // COMPUTE
        task.computeErrors(s, aT);

        // SET ERRORS
        for(j=0;j<3;j++) {
            if(!task.getActive(j)) continue;
            _pErr.append(task.getPositionError(j));
            _vErr.append(task.getVelocityError(j));
        }
    }
}
//_____________________________________________________________________________
/**
 * Compute the desired acceleration(s) for each task at a specified time.
 *
 * @param aT Time at which the desired accelerations are to be computed in
 * real time units.
 */
void CMC_TaskSet::
computeDesiredAccelerations(const SimTK::State& s, double aT)
{
    _w.setSize(0);
    _aDes.setSize(0);

    int i,j;
    for(i=0;i<getSize();i++) {

        // OBJECT
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        
        // COMPUTE
        task.computeDesiredAccelerations(s, aT);

        // SET WEIGHTS AND ACCELERATIONS
        for(j=0;j<3;j++) {
            if(!task.getActive(j)) continue;
            _w.append(task.getWeight(j));
            _aDes.append(task.getDesiredAcceleration(j));
        }
    }

    //printf("CMC_TaskSet.computeDesiredAccelerations: %d ",_aDes.size());
    //printf("track goals are active.\n");
}
//_____________________________________________________________________________
/**
 * Compute the desired acceleration(s) for each task given a time interval
 * over which the accelerations are to be achieved.
 *
 * Currently, the algorithm is to compute the position and velocity error
 * terms at the initial time and the acceleration term from the task set
 * at the final time.
 *
 * @param aTI Initial time of the time interval in real time units.
 * @param aTF Final time of the timer interval in real time units.
 */
void CMC_TaskSet::
computeDesiredAccelerations(const SimTK::State& s, double aTI,double aTF)
{
    _w.setSize(0);
    _aDes.setSize(0);

    int i,j;
    for(i=0;i<getSize();i++) {

        // OBJECT
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        
        // COMPUTE
        task.computeDesiredAccelerations(s, aTI,aTF);

        // SET WEIGHTS AND ACCELERATIONS
        for(j=0;j<3;j++) {
            if(!task.getActive(j)) continue;
            _w.append(task.getWeight(j));
            _aDes.append(task.getDesiredAcceleration(j));
        }
    }

    //printf("CMC_TaskSet.computeDesiredAccelerations: %d ",_aDes.size());
    //printf("track goals are active.\n");
}
//_____________________________________________________________________________
/**
 * Compute the acceleration(s) for each task.
 * Model::computeAccelerations must be called before this method
 * is called for the results to be valid.
 */
void CMC_TaskSet::
computeAccelerations(const SimTK::State& s)
{
    _a.setSize(0);

    int i,j;
    for(i=0;i<getSize();i++) {

        // OBJECT
        TrackingTask& ttask = get(i);

        // If CMC_Task process same way as pre 2.0.2
        if (dynamic_cast<CMC_Task*>(&ttask)==NULL) 
            continue;

        CMC_Task& task = dynamic_cast<CMC_Task&>(ttask);
        
        // COMPUTE
        task.computeAccelerations(s);

        // SET WEIGHTS AND ACCELERATIONS
        for(j=0;j<3;j++) {
            if(!task.getActive(j)) continue;
            _a.append(task.getAcceleration(j));
        }
    }

    //printf("CMC_TaskSet.computeAccelerations: %d ",_a.size());
    //printf("track goals are active.\n");
}


