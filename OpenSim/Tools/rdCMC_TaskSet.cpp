// rdCMC_TaskSet.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Copyright (c) 2006 Stanford University and Realistic Dynamics, Inc.
// Contributors: Frank C. Anderson
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS,
// CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
// THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <string>
#include <OpenSim/Common/PropertyObjArray.h>
#include "rdCMC_TaskSet.h"


using namespace std;
using namespace OpenSim;


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
rdCMC_TaskSet::~rdCMC_TaskSet()
{
}
//_____________________________________________________________________________
/**
 * Construct a default task set for a specified model.
 *
 * @param aModel Model for which tasks will be set.
 */
rdCMC_TaskSet::rdCMC_TaskSet() :
	_pTask(0.0),_vTask(0.0),_aTask(0.0),
	_pErrLast(0.0),_pErr(0.0),_vErrLast(0.0),_vErr(0.0),
	_kp(0.0),_kv(0.0),_ka(0.0),
	_w(0.0),_aDes(0.0),_a(0.0)
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Construct a task set from file.
 *
 * @param aFileName Name of the file.
 */
rdCMC_TaskSet::rdCMC_TaskSet(const string &aFileName) :
	Set<rdCMC_Task>(aFileName),
	_pTask(0.0),_vTask(0.0),_aTask(0.0),
	_pErrLast(0.0),_pErr(0.0),_vErrLast(0.0),_vErr(0.0),
	_kp(0.0),_kv(0.0),_ka(0.0),
	_w(0.0),_aDes(0.0),_a(0.0)
{
	setNull();
	updateFromXMLNode();
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void rdCMC_TaskSet::
setNull()
{
	setType("rdCMC_TaskSet");
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
void rdCMC_TaskSet::
setupProperties()
{
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
void rdCMC_TaskSet::
setModel(Model *aModel)
{
	_model = aModel;

	int i;
	rdCMC_Task *task;
	for(i=0;i<getSize();i++) {
		task = get(i);
		if(task==NULL) continue;
		task->setModel(_model);
	}
}

//_____________________________________________________________________________
/**
 * Get the model for which the tracking is performed.
 *
 * @return Pointer to the model.
 */
Model* rdCMC_TaskSet::
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
 * as rdCMC_Point tasks, the assumption is that there will be three
 * consecutive functions named "x" in the function set.  If the correct
 * number of functions is not found, the task is disabled.
 *
 * @param aFuncSet Function set.
 * @return Pointer to the previous function set.
 */
void rdCMC_TaskSet::
setFunctions(FunctionSet &aFuncSet)
{
	// LOOP THROUGH TRACK OBJECTS
	int i,j,iFunc=0;
	int nTrk;
	string name;
	Function *f[3];
	rdCMC_Task *task;
	for(i=0;i<getSize();i++) {

		// OBJECT
		task = get(i);
		if(task==NULL) continue;

		// NAME
		name = task->getName();
		if(name.empty()) continue;

		// FIND FUNCTION(S)
		f[0] = f[1] = f[2] = NULL;
		nTrk = task->getNumTaskFunctions();
		iFunc = aFuncSet.getIndex(name,iFunc);
		for(j=0;j<nTrk;j++) {
			try {
				f[j] = aFuncSet.get(iFunc);
			} catch(Exception x) {
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
		task->setTaskFunctions(f[0],f[1],f[2]);
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
 * as rdCMC_Point tasks, the assumption is that there will be three
 * consecutive functions named "x" in the function set.  If the correct
 * number of functions is not found, the task is disabled.
 *
 * @param aFuncSet Function set.
 * @return Pointer to the previous function set.
 */
void rdCMC_TaskSet::
setFunctionsForVelocity(FunctionSet &aFuncSet)
{
	// LOOP THROUGH TRACK OBJECTS
	int i,j,iFunc=0;
	int nTrk;
	string name;
	Function *f[3];
	rdCMC_Task *task;
	for(i=0;i<getSize();i++) {

		// OBJECT
		task = get(i);
		if(task==NULL) continue;

		// NAME
		name = task->getName();
		if(name.empty()) continue;

		// FIND FUNCTION(S)
		f[0] = f[1] = f[2] = NULL;
		nTrk = task->getNumTaskFunctions();
		iFunc = aFuncSet.getIndex(name,iFunc);
		for(j=0;j<nTrk;j++) {
			try {
				f[j] = aFuncSet.get(iFunc);
			} catch(Exception x) {
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
		task->setTaskFunctionsForVelocity(f[0],f[1],f[2]);
	}
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
 * as rdCMC_Point tasks, the assumption is that there will be three
 * consecutive functions named "x" in the function set.  If the correct
 * number of functions is not found, the task is disabled.
 *
 * @param aFuncSet Function set.
 * @return Pointer to the previous function set.
 */
void rdCMC_TaskSet::
setFunctionsForAcceleration(FunctionSet &aFuncSet)
{
	// LOOP THROUGH TRACK OBJECTS
	int i,j,iFunc=0;
	int nTrk;
	string name;
	Function *f[3];
	rdCMC_Task *task;
	for(i=0;i<getSize();i++) {

		// OBJECT
		task = get(i);
		if(task==NULL) continue;

		// NAME
		name = task->getName();
		if(name.empty()) continue;

		// FIND FUNCTION(S)
		f[0] = f[1] = f[2] = NULL;
		nTrk = task->getNumTaskFunctions();
		iFunc = aFuncSet.getIndex(name,iFunc);
		for(j=0;j<nTrk;j++) {
			try {
				f[j] = aFuncSet.get(iFunc);
			} catch(Exception x) {
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
		task->setTaskFunctionsForAcceleration(f[0],f[1],f[2]);
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
Array<double>& rdCMC_TaskSet::
getTaskPositions(double aT)
{
	_pTask.setSize(0);

	int i,j,n;
	int size = getSize();
	rdCMC_Task *task;
	for(i=0;i<size;i++) {
		task = get(i);
		if(task==NULL) continue;
		n = task->getNumTaskFunctions();
		for(j=0;j<n;j++) {
			if(!task->getActive(j)) continue;
			_pTask.append(task->getTaskPosition(j,aT));
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
Array<double>& rdCMC_TaskSet::
getTaskVelocities(double aT)
{
	_vTask.setSize(0);

	int i,j,n;
	int size = getSize();
	rdCMC_Task *task;
	for(i=0;i<size;i++) {
		task = get(i);
		if(task==NULL) continue;
		n = task->getNumTaskFunctions();
		for(j=0;j<n;j++) {
			if(!task->getActive(j)) continue;
			_vTask.append(task->getTaskVelocity(j,aT));
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
Array<double>& rdCMC_TaskSet::
getTaskAccelerations(double aT)
{
	_aTask.setSize(0);

	int i,j,n;
	int size = getSize();
	rdCMC_Task *task;
	for(i=0;i<size;i++) {
		task = get(i);
		if(task==NULL) continue;
		n = task->getNumTaskFunctions();
		for(j=0;j<n;j++) {
			if(!task->getActive(j)) continue;
			_aTask.append(task->getTaskAcceleration(j,aT));
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
Array<double>& rdCMC_TaskSet::
getPositionGains()
{
	_kp.setSize(0);

	int i,j,n;
	int size = getSize();
	rdCMC_Task *task;
	for(i=0;i<size;i++) {
		task = get(i);
		if(task==NULL) continue;
		n = task->getNumTaskFunctions();
		for(j=0;j<n;j++) {
			if(!task->getActive(j)) continue;
			_kp.append(task->getKP(j));
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
Array<double>& rdCMC_TaskSet::
getVelocityGains()
{
	_kv.setSize(0);

	int i,j,n;
	int size = getSize();
	rdCMC_Task *task;
	for(i=0;i<size;i++) {
		task = get(i);
		if(task==NULL) continue;
		n = task->getNumTaskFunctions();
		for(j=0;j<n;j++) {
			if(!task->getActive(j)) continue;
			_kv.append(task->getKV(j));
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
Array<double>& rdCMC_TaskSet::
getAccelerationGains()
{
	_ka.setSize(0);

	int i,j,n;
	int size = getSize();
	rdCMC_Task *task;
	for(i=0;i<size;i++) {
		task = get(i);
		if(task==NULL) continue;
		n = task->getNumTaskFunctions();
		for(j=0;j<n;j++) {
			if(!task->getActive(j)) continue;
			_ka.append(task->getKA(j));
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
Array<double>& rdCMC_TaskSet::
getPositionErrorsLast()
{
	return(_pErrLast);
}
//_____________________________________________________________________________
/**
 * Get the position errors.
 * @return Array of position errors.
 */
Array<double>& rdCMC_TaskSet::
getPositionErrors()
{
	return(_pErr);
}
//_____________________________________________________________________________
/**
 * Get the last velocity errors.
 * @return Array of last velocity errors.
 */
Array<double>& rdCMC_TaskSet::
getVelocityErrorsLast()
{
	return(_vErrLast);
}
//_____________________________________________________________________________
/**
 * Get the velocity errors.
 * @return Array of velocity errors.
 */
Array<double>& rdCMC_TaskSet::
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
Array<double>& rdCMC_TaskSet::
getWeights()
{
	_w.setSize(0);

	int i,j,n;
	int size = getSize();
	rdCMC_Task *task;
	for(i=0;i<size;i++) {
		task = get(i);
		if(task==NULL) continue;
		n = task->getNumTaskFunctions();
		for(j=0;j<n;j++) {
			if(!task->getActive(j)) continue;
			_w.append(task->getWeight(j));
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
Array<double>& rdCMC_TaskSet::
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
Array<double>& rdCMC_TaskSet::
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
void rdCMC_TaskSet::
recordErrorsAsLastErrors()
{
	_pErrLast.setSize(0);
	_vErrLast.setSize(0);

	int i,j;
	double e0,e1,e2;
	rdCMC_Task *task;
	for(i=0;i<getSize();i++) {

		// OBJECT
		task = get(i);
		if(task==NULL) continue;
		
		// POSITION ERRORS
		e0 = task->getPositionError(0);
		e1 = task->getPositionError(1);
		e2 = task->getPositionError(2);
		task->setPositionErrorLast(e0,e1,e2);

		// VELOCITY ERRORS
		e0 = task->getVelocityError(0);
		e1 = task->getVelocityError(1);
		e2 = task->getVelocityError(2);
		task->setVelocityErrorLast(e0,e1,e2);

		// SET ERRORS
		for(j=0;j<3;j++) {
			if(!task->getActive(j)) continue;
			_pErrLast.append(task->getPositionErrorLast(j));
			_vErrLast.append(task->getVelocityErrorLast(j));
		}
	}
}
//_____________________________________________________________________________
/**
 * Compute the errors for all tasks.
 *
 * @param aT Time at which to compute the errors in real time units.
 */
void rdCMC_TaskSet::
computeErrors(double aT)
{
	_pErr.setSize(0);
	_vErr.setSize(0);

	int i,j;
	rdCMC_Task *task;
	for(i=0;i<getSize();i++) {

		// OBJECT
		task = get(i);
		if(task==NULL) continue;
		
		// COMPUTE
		task->computeErrors(aT);

		// SET ERRORS
		for(j=0;j<3;j++) {
			if(!task->getActive(j)) continue;
			_pErr.append(task->getPositionError(j));
			_vErr.append(task->getVelocityError(j));
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
void rdCMC_TaskSet::
computeDesiredAccelerations(double aT)
{
	_w.setSize(0);
	_aDes.setSize(0);

	int i,j;
	rdCMC_Task *task;
	for(i=0;i<getSize();i++) {

		// OBJECT
		task = get(i);
		if(task==NULL) continue;
		
		// COMPUTE
		task->computeDesiredAccelerations(aT);

		// SET WEIGHTS AND ACCELERATIONS
		for(j=0;j<3;j++) {
			if(!task->getActive(j)) continue;
			_w.append(task->getWeight(j));
			_aDes.append(task->getDesiredAcceleration(j));
		}
	}

	//printf("rdCMC_TaskSet.computeDesiredAccelerations: %d ",_aDes.size());
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
void rdCMC_TaskSet::
computeDesiredAccelerations(double aTI,double aTF)
{
	_w.setSize(0);
	_aDes.setSize(0);

	int i,j;
	rdCMC_Task *task;
	for(i=0;i<getSize();i++) {

		// OBJECT
		task = get(i);
		if(task==NULL) continue;
		
		// COMPUTE
		task->computeDesiredAccelerations(aTI,aTF);

		// SET WEIGHTS AND ACCELERATIONS
		for(j=0;j<3;j++) {
			if(!task->getActive(j)) continue;
			_w.append(task->getWeight(j));
			_aDes.append(task->getDesiredAcceleration(j));
		}
	}

	//printf("rdCMC_TaskSet.computeDesiredAccelerations: %d ",_aDes.size());
	//printf("track goals are active.\n");
}
//_____________________________________________________________________________
/**
 * Compute the acceleration(s) for each task.
 * Model::computeAccelerations must be called before this method
 * is called for the results to be valid.
 */
void rdCMC_TaskSet::
computeAccelerations()
{
	_a.setSize(0);

	int i,j;
	rdCMC_Task *task;
	for(i=0;i<getSize();i++) {

		// OBJECT
		task = get(i);
		if(task==NULL) continue;
		
		// COMPUTE
		task->computeAccelerations();

		// SET WEIGHTS AND ACCELERATIONS
		for(j=0;j<3;j++) {
			if(!task->getActive(j)) continue;
			_a.append(task->getAcceleration(j));
		}
	}

	//printf("rdCMC_TaskSet.computeAccelerations: %d ",_a.size());
	//printf("track goals are active.\n");
}


