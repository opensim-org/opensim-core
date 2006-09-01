#ifndef _SimmIKSolverImpl_h_
#define _SimmIKSolverImpl_h_

#include <OpenSim/Applications/Workflow/workflowDLL.h>
#include <OpenSim/Simulation/SIMM/IKSolverInterface.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Tools/Storage.h>

namespace OpenSim { 

class rdOptimizationTarget;
class rdFSQP;
class SimmIKParams;
class SimmIKTrialParams;
class SimmInverseKinematicsTarget;

#ifdef SWIG
	#ifdef workflow_API
		#undef workflow_API
		#define workflow_API
	#endif
#endif
// SimmIKSolverImpl.h
// Author: Ayman Habib
/* Copyright (c) 2005, Stanford University and Ayman Habib
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

class workflow_API SimmIKSolverImpl : public IKSolverInterface 
{
public:
	SimmIKSolverImpl(SimmInverseKinematicsTarget&	aOptimizationTarget,
					const SimmIKParams&		aIKParams);

	virtual void solveFrames(const SimmIKTrialParams& aIKOptions, Storage& inputData, Storage& outputData);
private:

	void collectUserData(const Array<std::string> &,
						std::string& resultsHeader, 
						std::string& userHeaders, 
						Array<int>& userDataColumnIndices);

	void appendUserData(Array<double>& outputRow, 
						Array<int>& indices, 
						StateVector* inputRow);

};

}; //namespace

#endif // __SimmIKSolverImpl_h__



