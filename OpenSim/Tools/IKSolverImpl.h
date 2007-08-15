#ifndef _IKSolverImpl_h_
#define _IKSolverImpl_h_

#include "osimToolsDLL.h"
#include "IKSolverInterface.h"
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Common/Storage.h>

namespace SimTK {
	class Optimizer;
	class OptimizerSystem;
}

namespace OpenSim { 

class IKTrial;
class IKTarget;

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif
// IKSolverImpl.h
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

class OSIMTOOLS_API IKSolverImpl : public IKSolverInterface 
{
private:
	Array<int> _userDataColumnIndices;

public:
	IKSolverImpl(IKTarget& aOptimizationTarget);
	virtual ~IKSolverImpl() {}

	virtual void initializeSolver(const IKTrial& aIKOptions, Storage& inputData, Storage& outputData);
	virtual void solveFrames(const IKTrial& aIKOptions, Storage& inputData, Storage& outputData);
	virtual void interrupt();
private:

	void collectUserData(const Array<std::string> &,
						const Array<std::string> &resultColumnLabels,
						Array<std::string> &userColumnLabels,
						Array<int>& userDataColumnIndices);

	void appendUserData(Array<double>& outputRow, 
						Array<int>& indices, 
						StateVector* inputRow);

	SimTK::Optimizer *createOptimizer(const IKTrial &aIKOptions, SimTK::OptimizerSystem &aSystem) const;
};

}; //namespace

#endif // __IKSolverImpl_h__
