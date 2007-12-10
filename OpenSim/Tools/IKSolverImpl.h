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
/* Copyright (c)  2005, Stanford University and Ayman Habib
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
