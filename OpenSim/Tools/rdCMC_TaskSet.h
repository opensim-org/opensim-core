// rdCMC_TaskSet.h
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
#ifndef rdCMC_TaskSet_h__
#define rdCMC_TaskSet_h__

// INCLUDES
#include "osimToolsDLL.h"
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "rdCMC_Task.h"


#ifndef SWIG
template class OSIMTOOLS_API OpenSim::Set<OpenSim::rdCMC_Task>;
#endif

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * An class for holding and managing a set of tasks.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API rdCMC_TaskSet : public Set<rdCMC_Task> 
{

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

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	rdCMC_TaskSet();
	rdCMC_TaskSet(const std::string &aFileName);
	virtual ~rdCMC_TaskSet();
private:
	void setNull();
	void setupProperties();

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// MODEL
	void setModel(Model *aModel);
	Model* getModel() const;
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
	void computeErrors(double aT);
	void computeDesiredAccelerations(double aT);
	void computeDesiredAccelerations(double aTCurrent,double aTFuture);
	void computeAccelerations();


//=============================================================================
};	// END of class rdCMC_TaskSet
//=============================================================================
//=============================================================================

};  // end namespace

#endif // rdCMC_TaskSet_h__


