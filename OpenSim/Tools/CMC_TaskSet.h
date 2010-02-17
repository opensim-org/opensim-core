#ifndef CMC_TaskSet_h__
#define CMC_TaskSet_h__
// CMC_TaskSet.h
// Contributors: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// INCLUDES
#include "osimToolsDLL.h"
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "CMC_Task.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * An class for holding and managing a set of tasks.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API CMC_TaskSet : public Set<CMC_Task> 
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
	CMC_TaskSet();
	CMC_TaskSet(const std::string &aFileName);
	virtual ~CMC_TaskSet();
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
};	// END of class CMC_TaskSet
//=============================================================================
//=============================================================================

};  // end namespace

#endif // CMC_TaskSet_h__


